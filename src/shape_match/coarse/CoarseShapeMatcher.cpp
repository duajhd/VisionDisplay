#include "shape_match/coarse/CoarseShapeMatcher.h"

#include "shape_match/coarse/CoarseMatchReportWriter.h"
#include "shape_match/coarse/CoarseMatchProfiler.h"
#include "shape_match/coarse/CoarsePoseGenerator.h"
#include "shape_match/coarse/CandidateRegionGenerator.h"
#include "shape_match/coarse/ImagePyramid.h"
#include "shape_match/coarse/MissedTargetDiagnostic.h"
#include "shape_match/coarse/MissedTargetStageTrace.h"
#include "shape_match/coarse/OrientationVotingCandidateGenerator.h"
#include "shape_match/coarse/RoiDistanceFieldBuilder.h"
#include "shape_match/coarse/VotingConfig.h"
#include "shape_match/coarse/TemplatePyramid.h"
#include "shape_match/evaluation/CandidateRanker.h"
#include "shape_match/evaluation/MultiTargetEvaluator.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <unordered_set>

namespace ShapeMatch {

namespace {

double elapsedMsSince(std::chrono::steady_clock::time_point t0)
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

long long poseDiversityKey(const MatchPose& pose)
{
    const long long x = static_cast<long long>(std::llround(pose.x / 2.0));
    const long long y = static_cast<long long>(std::llround(pose.y / 2.0));
    const long long a = static_cast<long long>(std::llround(radToDeg(wrapToPi(pose.theta)) / 2.0));
    return (x << 42) ^ (y << 16) ^ (a & 0xffffLL);
}

int spatialCellIndex(const MatchPose& pose, cv::Size imageSize, int rows, int cols)
{
    const int col = std::clamp(static_cast<int>(pose.x / std::max(1.0, static_cast<double>(imageSize.width)) * cols), 0, cols - 1);
    const int row = std::clamp(static_cast<int>(pose.y / std::max(1.0, static_cast<double>(imageSize.height)) * rows), 0, rows - 1);
    return row * cols + col;
}

bool poseInRect(const MatchPose& pose, const cv::Rect& rect)
{
    return rect.contains(cv::Point(static_cast<int>(std::round(pose.x)), static_cast<int>(std::round(pose.y))));
}

bool poseTooCloseForFinalSelection(const MatchPose& a,
                                   const MatchPose& b,
                                   double translationThresholdPx,
                                   double angleThresholdDeg)
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    if (std::sqrt(dx * dx + dy * dy) > translationThresholdPx) {
        return false;
    }
    return std::abs(radToDeg(wrapToPi(a.theta - b.theta))) <= angleThresholdDeg;
}

std::vector<CoarseCandidate> selectFinalCoarseCandidates(const std::vector<CoarseCandidate>& candidates,
                                                         cv::Size imageSize,
                                                         const std::vector<cv::Rect>& activeRegions,
                                                         const CoarseMatchConfig& config)
{
    const int limit = std::max(1, config.finalTopK);
    std::vector<CoarseCandidate> selected;
    selected.reserve(static_cast<size_t>(std::min<int>(limit, candidates.size())));
    std::unordered_set<long long> seen;
    seen.reserve(static_cast<size_t>(limit * 2));
    const auto addCandidate = [&](const CoarseCandidate& candidate) {
        if (static_cast<int>(selected.size()) >= limit) {
            return false;
        }
        const long long key = poseDiversityKey(candidate.pose);
        if (!seen.insert(key).second) {
            return false;
        }
        selected.push_back(candidate);
        return true;
    };
    const auto addDiverseCandidate = [&](const CoarseCandidate& candidate) {
        for (const CoarseCandidate& existing : selected) {
            if (poseTooCloseForFinalSelection(candidate.pose,
                                             existing.pose,
                                             config.nmsTranslationThresholdPx,
                                             config.nmsAngleThresholdDeg)) {
                return false;
            }
        }
        return addCandidate(candidate);
    };

    for (const CoarseCandidate& candidate : candidates) {
        if (static_cast<int>(selected.size()) >= limit) {
            break;
        }
        addDiverseCandidate(candidate);
    }

    if (!activeRegions.empty()) {
        const int perRegion = std::max(2, std::min(8, limit / std::max(1, static_cast<int>(activeRegions.size())) + 1));
        for (const cv::Rect& region : activeRegions) {
            int kept = 0;
            for (const CoarseCandidate& candidate : candidates) {
                if (kept >= perRegion || static_cast<int>(selected.size()) >= limit) {
                    break;
                }
                if (poseInRect(candidate.pose, region) && addDiverseCandidate(candidate)) {
                    ++kept;
                }
            }
        }
    }

    const int rows = std::max(1, std::min(16, config.diversityGridRows));
    const int cols = std::max(1, std::min(16, config.diversityGridCols));
    const int perCell = std::max(2, std::min(6, limit / std::max(1, rows * cols) + 2));
    std::vector<int> cellCounts(static_cast<size_t>(rows * cols), 0);
    for (const CoarseCandidate& candidate : selected) {
        ++cellCounts[static_cast<size_t>(spatialCellIndex(candidate.pose, imageSize, rows, cols))];
    }
    for (const CoarseCandidate& candidate : candidates) {
        if (static_cast<int>(selected.size()) >= limit) {
            break;
        }
        const int cell = spatialCellIndex(candidate.pose, imageSize, rows, cols);
        if (cellCounts[static_cast<size_t>(cell)] >= perCell) {
            continue;
        }
        if (addDiverseCandidate(candidate)) {
            ++cellCounts[static_cast<size_t>(cell)];
        }
    }

    for (const CoarseCandidate& candidate : candidates) {
        if (static_cast<int>(selected.size()) >= limit) {
            break;
        }
        addCandidate(candidate);
    }
    return selected;
}

} // namespace

CoarseShapeMatcher::CoarseShapeMatcher(CoarseMatchConfig coarseConfig, ShapeMatchEvalConfig evalConfig)
    : m_coarseConfig(std::move(coarseConfig))
    , m_evalConfig(std::move(evalConfig))
{
    m_evalConfig.topK = std::max(1, m_coarseConfig.finalTopK);
    m_evalConfig.maxFinalRankerInputCandidates = std::max(m_evalConfig.maxFinalRankerInputCandidates, m_coarseConfig.finalTopK);
    m_evalConfig.targetFinalRankerInputCandidates = std::max(m_evalConfig.targetFinalRankerInputCandidates, m_coarseConfig.finalTopK);
}

CoarseMatchReport CoarseShapeMatcher::match(const ShapeTemplateModel& templateModel,
                                            const EdgeImageData& edgeData,
                                            const std::vector<GroundTruthInstance>* groundTruth) const
{
    const auto t0 = std::chrono::steady_clock::now();
    CoarseMatchReport report;
    if (groundTruth) {
        report.groundTruthInstances = *groundTruth;
    }

    if (templateModel.empty()) {
        report.failureReason = "empty_template_model";
        return report;
    }
    if (edgeData.empty()) {
        report.failureReason = "empty_edge_data";
        return report;
    }

    ImagePyramid imagePyramid;
    TemplatePyramid templatePyramid;
    const int requestedLevels = std::max(1, m_coarseConfig.pyramidLevels);
    auto sectionT0 = std::chrono::steady_clock::now();
    if (!imagePyramid.build(edgeData, requestedLevels, m_coarseConfig)) {
        report.failureReason = "image_pyramid_build_failed";
        return report;
    }
    report.profile.pyramidBuildTimeMs = elapsedMsSince(sectionT0);
    report.profile.distanceFieldBuildTimeMs = imagePyramid.totalDistanceFieldBuildTimeMs();
    sectionT0 = std::chrono::steady_clock::now();
    if (!templatePyramid.build(templateModel, imagePyramid.levelCount(), m_coarseConfig.maxTemplatePointsPerLevel)) {
        report.failureReason = "template_pyramid_build_failed";
        return report;
    }
    report.profile.templatePyramidBuildTimeMs = elapsedMsSince(sectionT0);

    CoarsePoseGenerator generator(m_coarseConfig);
    std::vector<CoarseCandidate> previous;
    RoiDistanceFieldSet level0RoiFields;
    std::vector<cv::Rect> level0ActiveRegions;
    for (int level = imagePyramid.levelCount() - 1; level >= 0; --level) {
        sectionT0 = std::chrono::steady_clock::now();
        CoarseMatchLevelResult levelResult;
        if (level == imagePyramid.levelCount() - 1) {
            if (m_coarseConfig.enableOrientationVoting) {
                VotingConfig votingConfig;
                votingConfig.enableOrientationVoting = true;
                votingConfig.topVotePeaks = std::max(1, m_coarseConfig.maxVotingCandidatesToScore);
                votingConfig.enableVotingDebugReport = m_coarseConfig.enableVotingReport;

                VotingDebugReport votingReport;
                OrientationVotingCandidateGenerator votingGenerator(votingConfig);
                const std::vector<MatchPose> votingCandidates = votingGenerator.generate(templatePyramid.level(level),
                                                                                         imagePyramid.level(level),
                                                                                         &votingReport);
                report.initialCandidateMode = "voting";
                report.votingRawCandidateCount = static_cast<int>(votingCandidates.size());
                report.votingTimeMs = votingReport.totalMs;
                report.profile.votingTimeMs += votingReport.totalMs;
                report.profile.buildRtTableMs += votingReport.buildRtTableMs;
                report.profile.sampleImageEdgesMs += votingReport.sampleImageEdgesMs;
                report.profile.votingAccumulatorMs += votingReport.votingMs;
                report.profile.peakFindMs += votingReport.peakFindMs;

                if (static_cast<int>(votingCandidates.size()) >= std::max(0, m_coarseConfig.minVotingCandidates)) {
                    levelResult = generator.searchVotingInitial(templatePyramid.level(level),
                                                                imagePyramid.level(level),
                                                                level,
                                                                votingReport,
                                                                votingCandidates);
                    report.profile.votingVerifyMs += levelResult.votingVerifyTimeMs;
                    report.votingScoredCandidateCount = levelResult.votingScoredCandidateCount;
                    report.votingBestScore = levelResult.votingBestScore;
                } else if (m_coarseConfig.fallbackToGridSearchIfVotingTooFew) {
                    report.initialCandidateMode = "grid_fallback";
                    report.gridFallbackUsed = true;
                    levelResult = generator.searchGlobal(templatePyramid.level(level), imagePyramid.level(level), level);
                    levelResult.gridFallbackUsed = true;
                    levelResult.votingRawCandidateCount = static_cast<int>(votingCandidates.size());
                    levelResult.votingTimeMs = votingReport.totalMs;
                    levelResult.buildRtTableMs = votingReport.buildRtTableMs;
                    levelResult.sampleImageEdgesMs = votingReport.sampleImageEdgesMs;
                    levelResult.votingAccumulatorMs = votingReport.votingMs;
                    levelResult.peakFindMs = votingReport.peakFindMs;
                    report.profile.globalSearchTimeMs += levelResult.elapsedMs;
                } else {
                    levelResult = generator.searchVotingInitial(templatePyramid.level(level),
                                                                imagePyramid.level(level),
                                                                level,
                                                                votingReport,
                                                                votingCandidates);
                    report.profile.votingVerifyMs += levelResult.votingVerifyTimeMs;
                    report.votingScoredCandidateCount = levelResult.votingScoredCandidateCount;
                    report.votingBestScore = levelResult.votingBestScore;
                }
            } else {
                report.initialCandidateMode = "grid";
                levelResult = generator.searchGlobal(templatePyramid.level(level), imagePyramid.level(level), level);
                report.profile.globalSearchTimeMs += elapsedMsSince(sectionT0);
            }
        } else {
            EdgeQueryContext levelQueryContext;
            const EdgeQueryContext* queryContextPtr = nullptr;
            const std::vector<cv::Rect>* activeRegionsPtr = nullptr;
            if (level == 0 && m_coarseConfig.roiDistanceField.enableRoiDistanceField
                && imagePyramid.skippedFullDistanceField(0)) {
                std::vector<CoarseCandidate> level0RegionSeeds;
                level0RegionSeeds.reserve(previous.size());
                for (const CoarseCandidate& parent : previous) {
                    CoarseCandidate seed = parent;
                    seed.pose = CoarsePoseGenerator::convertPoseBetweenLevels(parent.pose,
                                                                              imagePyramid.scaleOfLevel(level + 1),
                                                                              imagePyramid.scaleOfLevel(level));
                    level0RegionSeeds.push_back(std::move(seed));
                }
                CandidateRegionGenerator regionGenerator(m_coarseConfig);
                const std::vector<CandidateRegion> regions = regionGenerator.generateLevelRegions(level0RegionSeeds,
                                                                                                  templatePyramid.level(level),
                                                                                                  imagePyramid.level(level).imageSize,
                                                                                                  level);
                level0ActiveRegions.clear();
                level0ActiveRegions.reserve(regions.size());
                for (const CandidateRegion& region : regions) {
                    level0ActiveRegions.push_back(region.roi);
                }
                RoiDistanceFieldBuilder roiBuilder;
                const auto roiT0 = std::chrono::steady_clock::now();
                level0RoiFields = roiBuilder.buildForRegions(imagePyramid.level(level),
                                                             level0ActiveRegions,
                                                             level,
                                                             imagePyramid.scaleOfLevel(level),
                                                             m_coarseConfig.roiDistanceField);
                report.profile.roiDistanceFieldBuildTimeMs = elapsedMsSince(roiT0);
                report.profile.roiDistanceFieldCount = level0RoiFields.size();
                report.profile.roiDistanceFieldTotalPixels = level0RoiFields.totalPixels();
                report.roiDistanceFieldRows.clear();
                for (const RoiDistanceField& field : level0RoiFields.fields()) {
                    RoiDistanceFieldProfileRow row;
                    row.level = field.level;
                    row.roiId = field.id;
                    row.x = field.roi.x;
                    row.y = field.roi.y;
                    row.width = field.roi.width;
                    row.height = field.roi.height;
                    row.pixelCount = field.pixelCount;
                    row.buildTimeMs = field.buildTimeMs;
                    row.valid = field.valid;
                    row.sourceCandidateCount = field.sourceCandidateCount;
                    report.roiDistanceFieldRows.push_back(row);
                }
                levelQueryContext.fullEdgeData = &imagePyramid.level(level);
                levelQueryContext.roiFields = &level0RoiFields;
                levelQueryContext.preferRoiField = true;
                levelQueryContext.allowFullField = m_coarseConfig.roiDistanceField.allowFullLevel0DistanceFieldFallback;
                levelQueryContext.allowLocalWindowFallback = m_coarseConfig.roiDistanceField.fallbackToLocalWindowIfNoRoiField;
                queryContextPtr = &levelQueryContext;
                activeRegionsPtr = &level0ActiveRegions;
            }
            levelResult = generator.searchLocal(templatePyramid.level(level),
                                                imagePyramid.level(level),
                                                level,
                                                imagePyramid.scaleOfLevel(level),
                                                imagePyramid.scaleOfLevel(level + 1),
                                                previous,
                                                queryContextPtr,
                                                activeRegionsPtr);
            report.profile.localRefineTimeMs += elapsedMsSince(sectionT0);
        }
        levelResult.distanceFieldBuildTimeMs = imagePyramid.distanceFieldBuildTimeMs(level);
        report.profile.fastScoreTimeMs += levelResult.fastScoreTimeMs;
        report.profile.nmsTimeMs += levelResult.nmsTimeMs;
        report.profile.upperBoundPrunedCount += levelResult.upperBoundPrunedCount;
        report.profile.totalEvaluatedPoints += levelResult.totalEvaluatedPoints;
        report.profile.totalEvaluatedCandidates += levelResult.evaluatedCandidateCount;
        report.profile.soaBuildTimeMs += levelResult.soaBuildTimeMs;
        report.profile.rotatedCacheBuildTimeMs += levelResult.rotatedCacheBuildTimeMs;
        report.profile.parallelScoringTimeMs += levelResult.parallelScoringTimeMs;
        report.profile.mergeTimeMs += levelResult.mergeTimeMs;
        if (level == 0) {
            report.profile.level0CandidateCountBefore = levelResult.rawGeneratedCandidates;
            report.profile.level0CandidateCountAfterBudget = levelResult.afterBudgetCandidates;
            report.profile.level0ScoredCandidateCount = levelResult.evaluatedCandidateCount;
            report.profile.level0ParentBefore = levelResult.parentCountBeforeSelection;
            report.profile.level0ParentAfter = levelResult.parentCountAfterSelection;
            report.profile.level0RawChildren = levelResult.rawGeneratedCandidates;
            report.profile.level0ScoredChildren = levelResult.evaluatedCandidateCount;
        }
        previous = levelResult.topCandidates;
        report.levels.push_back(std::move(levelResult));
        if (previous.empty()) {
            report.failureReason = "no_candidates_at_level_" + std::to_string(level);
            break;
        }
    }

    if (!previous.empty() && report.failureReason.empty()) {
        report.finalCoarseCandidates = selectFinalCoarseCandidates(previous,
                                                                   imagePyramid.level(0).imageSize,
                                                                   level0ActiveRegions,
                                                                   m_coarseConfig);
        const int n = std::min<int>(m_coarseConfig.finalTopK, static_cast<int>(report.finalCoarseCandidates.size()));
        report.finalPoses.reserve(static_cast<size_t>(n));
        for (int i = 0; i < n; ++i) {
            report.finalPoses.push_back(report.finalCoarseCandidates[static_cast<size_t>(i)].pose);
        }

        sectionT0 = std::chrono::steady_clock::now();
        CandidateRanker ranker(m_evalConfig);
        const EdgeImageData& finalEdgeData = imagePyramid.level(0);
        if (m_coarseConfig.roiDistanceField.enableRoiDistanceField && level0RoiFields.size() > 0) {
            EdgeQueryContext finalQueryContext;
            finalQueryContext.fullEdgeData = &finalEdgeData;
            finalQueryContext.roiFields = &level0RoiFields;
            finalQueryContext.preferRoiField = true;
            finalQueryContext.allowFullField = finalEdgeData.hasNearestEdgeField();
            finalQueryContext.allowLocalWindowFallback = m_coarseConfig.roiDistanceField.fallbackToLocalWindowIfNoRoiField;
            report.finalRankedCandidates = ranker.rank(report.finalPoses,
                                                       templateModel,
                                                       finalQueryContext,
                                                       groundTruth,
                                                       &report.profile.finalRankerProfile);
        } else {
            report.finalRankedCandidates = ranker.rank(report.finalPoses,
                                                       templateModel,
                                                       finalEdgeData,
                                                       groundTruth,
                                                       &report.profile.finalRankerProfile);
        }
        report.profile.finalRankerTimeMs = elapsedMsSince(sectionT0);
        report.profile.finalRankerProfile.totalFinalRankerTimeMs = report.profile.finalRankerTimeMs;
        if (groundTruth && !groundTruth->empty()) {
            MultiTargetEvaluator multi(m_evalConfig);
            report.multiTargetEval = multi.evaluate(report.finalRankedCandidates, *groundTruth);
        }
    }

    report.totalTimeMs = elapsedMsSince(t0);
    report.profile.totalTimeMs = report.totalTimeMs;
    report.profile.largeImageMode = m_coarseConfig.roiDistanceField.enableRoiDistanceField
        && imagePyramid.levelCount() > 0
        && imagePyramid.skippedFullDistanceField(0);
    report.profile.skippedFullLevel0DistanceField = imagePyramid.levelCount() > 0 && imagePyramid.skippedFullDistanceField(0);
    report.profile.avgEvaluatedPointsPerCandidate = report.profile.totalEvaluatedCandidates > 0
        ? static_cast<double>(report.profile.totalEvaluatedPoints) / static_cast<double>(report.profile.totalEvaluatedCandidates)
        : 0.0;
    report.profile.candidateReductionRatio = report.profile.level0CandidateCountBefore > 0
        ? static_cast<double>(report.profile.level0CandidateCountAfterBudget) / static_cast<double>(report.profile.level0CandidateCountBefore)
        : 1.0;
    const std::filesystem::path reportsDir = std::filesystem::path("data") / "shape_match" / "reports";
    sectionT0 = std::chrono::steady_clock::now();
    CoarseMatchReportWriter writer;
    writer.writeAll(report, m_coarseConfig, reportsDir);
    report.profile.reportWriteTimeMs = elapsedMsSince(sectionT0);
    CoarseMatchProfiler profiler;
    profiler.writeAll(report, m_coarseConfig, reportsDir);
    if (groundTruth && !groundTruth->empty() && imagePyramid.levelCount() == templatePyramid.levelCount()) {
        MissedTargetDiagnostic missed;
        missed.writeAll(report, m_coarseConfig, imagePyramid, templatePyramid, reportsDir);
        if (m_coarseConfig.enableMissedTargetStageTrace) {
            MissedTargetStageTraceWriter stageTrace;
            stageTrace.writeAll(report, m_coarseConfig, imagePyramid, templatePyramid, reportsDir);
        }
    }
    return report;
}

} // namespace ShapeMatch
