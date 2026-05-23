#include "shape_match/coarse/CoarsePoseGenerator.h"

#include "shape_match/coarse/CandidateBudgetPolicy.h"
#include "shape_match/coarse/ParallelCandidateScorer.h"
#include "shape_match/coarse/SpatialDiversityTopKBuffer.h"
#include "shape_match/coarse/RotatedTemplateCache.h"
#include "shape_match/coarse/TemplatePointSoA.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <unordered_set>
#include <utility>

namespace ShapeMatch {

namespace {

double elapsedMsSince(std::chrono::steady_clock::time_point t0)
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

std::pair<double, int> voteInfoForIndex(const VotingDebugReport& report, size_t index)
{
    if (index >= report.topPeaks.size()) {
        return {0.0, 0};
    }
    return {static_cast<double>(report.topPeaks[index].voteScore), report.topPeaks[index].voteCount};
}

bool poseCenterInSearchRoi(const CoarseMatchConfig& config, const MatchPose& pose, int level)
{
    if (!config.enableSearchRoi || config.searchRoiWidth <= 0 || config.searchRoiHeight <= 0) {
        return true;
    }
    const double scale = std::pow(0.5, std::max(0, level));
    const double x0 = static_cast<double>(config.searchRoiX) * scale;
    const double y0 = static_cast<double>(config.searchRoiY) * scale;
    const double x1 = static_cast<double>(config.searchRoiX + config.searchRoiWidth) * scale;
    const double y1 = static_cast<double>(config.searchRoiY + config.searchRoiHeight) * scale;
    return pose.x >= x0 && pose.y >= y0 && pose.x < x1 && pose.y < y1;
}

bool poseCenterInRegions(const MatchPose& pose, const std::vector<cv::Rect>* regions)
{
    if (!regions || regions->empty()) {
        return true;
    }
    const cv::Point p(static_cast<int>(std::round(pose.x)), static_cast<int>(std::round(pose.y)));
    for (const cv::Rect& region : *regions) {
        if (region.contains(p)) {
            return true;
        }
    }
    return false;
}

FastScoreContext scoreContext(const CoarseMatchConfig& config,
                              const SpatialDiversityTopKBuffer& diversityBuffer,
                              const TopKCandidateBuffer& legacyBuffer)
{
    FastScoreContext context;
    const bool enough = config.enableSpatialDiversity ? diversityBuffer.hasEnough() : legacyBuffer.hasEnough();
    if (enough) {
        context.hasTopKThreshold = true;
        context.currentTopKMinScore = static_cast<float>(config.enableSpatialDiversity
            ? diversityBuffer.minScore()
            : legacyBuffer.minScore());
    }
    return context;
}

void accumulateScoreStats(CoarseMatchLevelResult& result, const CoarseCandidate& candidate)
{
    result.totalEvaluatedPoints += candidate.evaluatedPoints;
    if (candidate.rejectedByEarlyExit) {
        ++result.earlyExitCandidateCount;
        ++result.prunedCandidateCount;
        if (candidate.rejectReason == "upper_bound_pruned") {
            ++result.upperBoundPrunedCount;
        }
    }
    if (candidate.evaluatedPoints >= candidate.totalPointCount && candidate.totalPointCount > 0) {
        ++result.fullEvaluatedCandidateCount;
    }
}

void finalizeLevelStats(CoarseMatchLevelResult& result)
{
    result.avgEvaluatedPointsPerCandidate = result.evaluatedCandidateCount > 0
        ? static_cast<double>(result.totalEvaluatedPoints) / static_cast<double>(result.evaluatedCandidateCount)
        : 0.0;
    result.scoredCandidates = result.evaluatedCandidateCount;
    result.candidateReductionRatio = result.rawGeneratedCandidates > 0
        ? static_cast<double>(result.afterBudgetCandidates) / static_cast<double>(result.rawGeneratedCandidates)
        : 1.0;
}

CandidateTraceInfo traceInfoFromCandidate(const CoarseCandidate& candidate,
                                          const std::string& stage,
                                          int level,
                                          int rank)
{
    CandidateTraceInfo info;
    info.candidateId = candidate.traceCandidateId;
    info.source = stage;
    info.level = level;
    info.rank = rank;
    info.pose = candidate.pose;
    info.voteScore = candidate.voteScore;
    info.voteCount = candidate.voteCount;
    info.fastScore = candidate.fastScore;
    info.coverageApprox = candidate.coverageApprox;
    info.orientationApprox = candidate.orientationApprox;
    info.polarityApprox = candidate.polarityApprox;
    info.exists = true;
    info.kept = true;
    info.suppressedByNms = candidate.suppressedByNms;
    info.suppressorCandidateId = candidate.suppressorCandidateId;
    info.prunedByUpperBound = candidate.rejectedByEarlyExit && candidate.rejectReason == "upper_bound_pruned";
    info.rejectReason = candidate.rejectReason;
    info.gridCellRow = candidate.sourceCellRow;
    info.gridCellCol = candidate.sourceCellCol;
    return info;
}

void appendTraceStage(CoarseMatchLevelResult& result,
                      const CoarseMatchConfig& config,
                      const std::string& stage,
                      const std::vector<CoarseCandidate>& candidates)
{
    if (!config.enableStageTrace) {
        return;
    }
    StageTraceSnapshot snapshot;
    snapshot.stage = stage;
    snapshot.level = result.level;
    const int limit = std::min(std::max(0, config.maxTraceCandidatesPerStage), static_cast<int>(candidates.size()));
    snapshot.candidates.reserve(static_cast<size_t>(limit));
    for (int i = 0; i < limit; ++i) {
        snapshot.candidates.push_back(traceInfoFromCandidate(candidates[static_cast<size_t>(i)], stage, result.level, i));
    }
    result.stageTrace.push_back(std::move(snapshot));
}

void appendTraceStageFromPoses(CoarseMatchLevelResult& result,
                               const CoarseMatchConfig& config,
                               const std::string& stage,
                               const std::vector<MatchPose>& poses,
                               const std::string& rejectReason = {})
{
    if (!config.enableStageTrace) {
        return;
    }
    StageTraceSnapshot snapshot;
    snapshot.stage = stage;
    snapshot.level = result.level;
    const int limit = std::min(std::max(0, config.maxTraceCandidatesPerStage), static_cast<int>(poses.size()));
    snapshot.candidates.reserve(static_cast<size_t>(limit));
    for (int i = 0; i < limit; ++i) {
        CandidateTraceInfo info;
        info.candidateId = i;
        info.source = stage;
        info.level = result.level;
        info.rank = i;
        info.pose = poses[static_cast<size_t>(i)];
        info.exists = true;
        info.kept = rejectReason.empty();
        info.clippedByBudget = rejectReason == "budget_clipped";
        info.droppedByParentSelection = rejectReason == "parent_selection_dropped";
        info.rejectReason = rejectReason;
        snapshot.candidates.push_back(std::move(info));
    }
    result.stageTrace.push_back(std::move(snapshot));
}

long long poseKey(const MatchPose& pose)
{
    const long long xi = static_cast<long long>(std::llround(pose.x)) & 0x1fffffLL;
    const long long yi = static_cast<long long>(std::llround(pose.y)) & 0x1fffffLL;
    const long long ai = (static_cast<long long>(std::llround(radToDeg(wrapToPi(pose.theta)) * 10.0)) + 3600LL) & 0x3fffLL;
    return (xi << 35) ^ (yi << 14) ^ ai;
}

double cacheStepForLevel(const CoarseMatchConfig& config, int level)
{
    if (level <= 0) {
        return std::max(0.25, config.rotatedCacheAngleStepDeg);
    }
    return std::max(config.rotatedCacheAngleStepDeg, std::min(config.coarseAngleStepDeg, 2.0));
}

struct LocalOffset
{
    int dx = 0;
    int dy = 0;
    double da = 0.0;
    double cost = 0.0;
};

std::vector<int> centeredIntegerValues(int radius, int step)
{
    std::vector<int> values{0};
    const int s = std::max(1, step);
    for (int v = s; v <= radius; v += s) {
        values.push_back(v);
        values.push_back(-v);
    }
    if (radius > 0) {
        values.push_back(radius);
        values.push_back(-radius);
    }
    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end()), values.end());
    return values;
}

std::vector<double> centeredAngleValues(double radius, double step)
{
    std::vector<double> values{0.0};
    const double s = std::max(0.1, step);
    for (double v = s; v <= radius + 1e-9; v += s) {
        values.push_back(v);
        values.push_back(-v);
    }
    if (radius > 1e-9) {
        values.push_back(radius);
        values.push_back(-radius);
    }
    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end(), [](double a, double b) {
        return std::abs(a - b) < 1e-9;
    }), values.end());
    return values;
}

std::vector<LocalOffset> buildLocalOffsets(const RefineWindow& window, int translationStep, double angleStep)
{
    std::vector<LocalOffset> offsets;
    const std::vector<int> xyValues = centeredIntegerValues(window.radiusPx, translationStep);
    const std::vector<double> angleValues = centeredAngleValues(window.angleRadiusDeg, angleStep);
    for (double da : angleValues) {
        for (int dy : xyValues) {
            for (int dx : xyValues) {
                LocalOffset offset;
                offset.dx = dx;
                offset.dy = dy;
                offset.da = da;
                offset.cost = std::hypot(static_cast<double>(dx), static_cast<double>(dy)) + 0.25 * std::abs(da);
                offsets.push_back(offset);
            }
        }
    }
    std::stable_sort(offsets.begin(), offsets.end(), [](const LocalOffset& a, const LocalOffset& b) {
        return a.cost < b.cost;
    });
    return offsets;
}

} // namespace

CoarsePoseGenerator::CoarsePoseGenerator(CoarseMatchConfig config)
    : m_config(std::move(config))
{
}

CoarseMatchLevelResult CoarsePoseGenerator::searchGlobal(const ShapeTemplateModel& templateLevel,
                                                         const EdgeImageData& imageLevel,
                                                         int level) const
{
    const auto t0 = std::chrono::steady_clock::now();
    CoarseMatchLevelResult result;
    result.level = level;
    result.imageWidth = imageLevel.imageSize.width;
    result.imageHeight = imageLevel.imageSize.height;
    result.templatePoints = templateLevel.pointCount();
    result.angleStepDeg = angleStepForLevel(level);
    result.translationStepPx = translationStepForLevel(level);
    result.candidateSource = "grid";
    result.distanceFieldValid = imageLevel.hasDistanceField;
    result.scoringMode = (m_config.enableDistanceFieldScoring && imageLevel.hasDistanceField)
        ? "distance_field"
        : "local_search";
    result.localSearchFallbackCount = result.distanceFieldValid ? 0 : 1;

    TopKCandidateBuffer legacyBuffer(m_config.topKPerLevel,
                                     std::max(m_config.nmsTranslationThresholdPx, static_cast<double>(result.translationStepPx)),
                                     std::max(m_config.nmsAngleThresholdDeg, result.angleStepDeg * 0.75));
    SpatialDiversityTopKBuffer diversityBuffer(m_config,
                                               imageLevel.imageSize.width,
                                               imageLevel.imageSize.height,
                                               m_config.topKPerLevel,
                                               std::max(m_config.nmsTranslationThresholdPx, static_cast<double>(result.translationStepPx)),
                                               std::max(m_config.nmsAngleThresholdDeg, result.angleStepDeg * 0.75));
    FastPoseScorer scorer(m_config);
    const std::vector<double> scales = scaleValues();
    std::vector<CoarseCandidate> scoredTrace;
    scoredTrace.reserve(static_cast<size_t>(std::max(0, m_config.maxTraceCandidatesPerStage)));
    int traceId = 0;
    for (double angle = m_config.minAngleDeg; angle <= m_config.maxAngleDeg + 1e-9; angle += result.angleStepDeg) {
        const auto angleT0 = std::chrono::steady_clock::now();
        CoarseMatchAngleStat angleStat;
        angleStat.angleDeg = angle;
        for (double scale : scales) {
            for (int y = 0; y < imageLevel.imageSize.height; y += result.translationStepPx) {
                for (int x = 0; x < imageLevel.imageSize.width; x += result.translationStepPx) {
                    if (result.evaluatedCandidateCount >= m_config.maxCandidatesEvaluatedPerLevel) {
                        result.candidateLimitHit = true;
                        const auto nmsT0 = std::chrono::steady_clock::now();
                        result.topCandidates = m_config.enableSpatialDiversity
                            ? diversityBuffer.sortedCandidates()
                            : legacyBuffer.sortedCandidates();
                        result.nmsTimeMs += elapsedMsSince(nmsT0);
                        result.keptCandidateCount = static_cast<int>(result.topCandidates.size());
                        result.nmsBeforeCount = m_config.enableSpatialDiversity ? diversityBuffer.nmsBeforeCount() : result.keptCandidateCount;
                        result.nmsAfterCount = m_config.enableSpatialDiversity ? diversityBuffer.nmsAfterCount() : result.keptCandidateCount;
                        result.cellStats = m_config.enableSpatialDiversity ? diversityBuffer.cellStats() : std::vector<CoarseMatchCellStat>();
                        finalizeLevelStats(result);
                        result.elapsedMs = elapsedMsSince(t0);
                        return result;
                    }
                    const MatchPose pose = MatchPose::fromDeg(static_cast<double>(x), static_cast<double>(y), angle, scale);
                    if (!poseCenterInSearchRoi(m_config, pose, level)) {
                        ++result.generatedCandidateCount;
                        continue;
                    }
                    const FastScoreContext context = scoreContext(m_config, diversityBuffer, legacyBuffer);
                    const auto scoreT0 = std::chrono::steady_clock::now();
                    CoarseCandidate candidate = scorer.scorePose(templateLevel, imageLevel, pose, level, context);
                    candidate.traceCandidateId = traceId++;
                    candidate.source = "grid";
                    result.fastScoreTimeMs += elapsedMsSince(scoreT0);
                    ++result.evaluatedCandidateCount;
                    ++result.generatedCandidateCount;
                    ++angleStat.evaluatedCandidateCount;
                    accumulateScoreStats(result, candidate);
                    if (m_config.enableStageTrace && static_cast<int>(scoredTrace.size()) < m_config.maxTraceCandidatesPerStage) {
                        scoredTrace.push_back(candidate);
                    }
                    angleStat.bestFastScore = std::max(angleStat.bestFastScore, candidate.fastScore);
                    if (candidate.fastScore >= m_config.minFastScoreForPropagation) {
                        if (m_config.enableSpatialDiversity) {
                            diversityBuffer.add(candidate);
                        } else {
                            legacyBuffer.add(candidate);
                        }
                    }
                }
            }
        }
        angleStat.elapsedMs = elapsedMsSince(angleT0);
        result.angleStats.push_back(angleStat);
    }

    const auto nmsT0 = std::chrono::steady_clock::now();
    appendTraceStage(result, m_config, "level_initial_candidates", scoredTrace);
    appendTraceStage(result, m_config, "after_fast_scoring", scoredTrace);
    std::vector<CoarseCandidate> afterPruningTrace;
    afterPruningTrace.reserve(scoredTrace.size());
    for (const CoarseCandidate& candidate : scoredTrace) {
        if (!candidate.rejectedByEarlyExit) {
            afterPruningTrace.push_back(candidate);
        }
    }
    appendTraceStage(result, m_config, "after_upper_bound_pruning", afterPruningTrace);
    result.topCandidates = m_config.enableSpatialDiversity
        ? diversityBuffer.sortedCandidates()
        : legacyBuffer.sortedCandidates();
    result.nmsTimeMs += elapsedMsSince(nmsT0);
    result.keptCandidateCount = static_cast<int>(result.topCandidates.size());
    result.nmsBeforeCount = m_config.enableSpatialDiversity ? diversityBuffer.nmsBeforeCount() : result.keptCandidateCount;
    result.nmsAfterCount = m_config.enableSpatialDiversity ? diversityBuffer.nmsAfterCount() : result.keptCandidateCount;
    result.cellStats = m_config.enableSpatialDiversity ? diversityBuffer.cellStats() : std::vector<CoarseMatchCellStat>();
    result.bestFastScore = result.topCandidates.empty() ? 0.0 : result.topCandidates.front().fastScore;
    result.worstKeptFastScore = result.topCandidates.empty() ? 0.0 : result.topCandidates.back().fastScore;
    if (m_config.traceSuppressedCandidates) {
        appendTraceStage(result,
                         m_config,
                         "nms_suppressed_candidates",
                         m_config.enableSpatialDiversity ? diversityBuffer.suppressedCandidates() : legacyBuffer.suppressedCandidates());
    }
    appendTraceStage(result, m_config, "after_nms", result.topCandidates);
    appendTraceStage(result, m_config, "after_spatial_diversity", result.topCandidates);
    appendTraceStage(result, m_config, "level_output_beam", result.topCandidates);
    finalizeLevelStats(result);
    result.elapsedMs = elapsedMsSince(t0);
    return result;
}

CoarseMatchLevelResult CoarsePoseGenerator::searchVotingInitial(const ShapeTemplateModel& templateLevel,
                                                                const EdgeImageData& imageLevel,
                                                                int level,
                                                                const VotingDebugReport& votingReport,
                                                                const std::vector<MatchPose>& votingPoses) const
{
    const auto t0 = std::chrono::steady_clock::now();
    CoarseMatchLevelResult result;
    result.level = level;
    result.imageWidth = imageLevel.imageSize.width;
    result.imageHeight = imageLevel.imageSize.height;
    result.templatePoints = templateLevel.pointCount();
    result.angleStepDeg = angleStepForLevel(level);
    result.translationStepPx = translationStepForLevel(level);
    result.candidateSource = "voting";
    result.distanceFieldValid = imageLevel.hasDistanceField;
    result.votingRawCandidateCount = static_cast<int>(votingPoses.size());
    result.votingTimeMs = votingReport.totalMs;
    result.buildRtTableMs = votingReport.buildRtTableMs;
    result.sampleImageEdgesMs = votingReport.sampleImageEdgesMs;
    result.votingAccumulatorMs = votingReport.votingMs;
    result.peakFindMs = votingReport.peakFindMs;
    result.scoringMode = (m_config.enableDistanceFieldScoring && imageLevel.hasDistanceField)
        ? "distance_field"
        : "local_search";
    result.localSearchFallbackCount = result.distanceFieldValid ? 0 : 1;

    TopKCandidateBuffer legacyBuffer(m_config.topKPerLevel,
                                     std::max(m_config.nmsTranslationThresholdPx, static_cast<double>(result.translationStepPx)),
                                     std::max(m_config.nmsAngleThresholdDeg, result.angleStepDeg * 0.75));
    SpatialDiversityTopKBuffer diversityBuffer(m_config,
                                               imageLevel.imageSize.width,
                                               imageLevel.imageSize.height,
                                               m_config.topKPerLevel,
                                               std::max(m_config.nmsTranslationThresholdPx, static_cast<double>(result.translationStepPx)),
                                               std::max(m_config.nmsAngleThresholdDeg, result.angleStepDeg * 0.75));
    FastPoseScorer scorer(m_config);
    const int limit = std::min<int>(std::max(0, m_config.maxVotingCandidatesToScore),
                                    static_cast<int>(votingPoses.size()));
    if (m_config.enableStageTrace) {
        appendTraceStageFromPoses(result, m_config, "voting_raw_peaks", votingPoses);
    }
    std::vector<CoarseCandidate> scoredTrace;
    scoredTrace.reserve(static_cast<size_t>(std::min(limit, std::max(0, m_config.maxTraceCandidatesPerStage))));
    int traceId = 0;
    for (int i = 0; i < limit; ++i) {
        if (result.evaluatedCandidateCount >= m_config.maxCandidatesEvaluatedPerLevel) {
            result.candidateLimitHit = true;
            break;
        }
        if (!poseCenterInSearchRoi(m_config, votingPoses[static_cast<size_t>(i)], level)) {
            ++result.generatedCandidateCount;
            continue;
        }
        const FastScoreContext context = scoreContext(m_config, diversityBuffer, legacyBuffer);
        const auto scoreT0 = std::chrono::steady_clock::now();
        CoarseCandidate candidate = scorer.scorePose(templateLevel, imageLevel, votingPoses[static_cast<size_t>(i)], level, context);
        candidate.traceCandidateId = traceId++;
        result.fastScoreTimeMs += elapsedMsSince(scoreT0);
        const auto voteInfo = voteInfoForIndex(votingReport, static_cast<size_t>(i));
        candidate.source = "voting";
        candidate.voteScore = voteInfo.first;
        candidate.voteCount = voteInfo.second;
        ++result.evaluatedCandidateCount;
        ++result.generatedCandidateCount;
        ++result.votingScoredCandidateCount;
        accumulateScoreStats(result, candidate);
        if (m_config.enableStageTrace && static_cast<int>(scoredTrace.size()) < m_config.maxTraceCandidatesPerStage) {
            scoredTrace.push_back(candidate);
        }
        result.votingBestScore = std::max(result.votingBestScore, candidate.fastScore);
        if (candidate.fastScore >= m_config.minFastScoreForPropagation) {
            ++result.votingAcceptedCandidateCount;
            if (m_config.enableSpatialDiversity) {
                diversityBuffer.add(candidate);
            } else {
                legacyBuffer.add(candidate);
            }
        }
    }
    appendTraceStage(result, m_config, "voting_verified", scoredTrace);
    appendTraceStage(result, m_config, "after_fast_scoring", scoredTrace);
    std::vector<CoarseCandidate> afterPruningTrace;
    afterPruningTrace.reserve(scoredTrace.size());
    for (const CoarseCandidate& candidate : scoredTrace) {
        if (!candidate.rejectedByEarlyExit) {
            afterPruningTrace.push_back(candidate);
        }
    }
    appendTraceStage(result, m_config, "after_upper_bound_pruning", afterPruningTrace);

    const auto nmsT0 = std::chrono::steady_clock::now();
    result.topCandidates = m_config.enableSpatialDiversity
        ? diversityBuffer.sortedCandidates()
        : legacyBuffer.sortedCandidates();
    result.nmsTimeMs += elapsedMsSince(nmsT0);
    result.keptCandidateCount = static_cast<int>(result.topCandidates.size());
    result.nmsBeforeCount = m_config.enableSpatialDiversity ? diversityBuffer.nmsBeforeCount() : result.keptCandidateCount;
    result.nmsAfterCount = m_config.enableSpatialDiversity ? diversityBuffer.nmsAfterCount() : result.keptCandidateCount;
    result.cellStats = m_config.enableSpatialDiversity ? diversityBuffer.cellStats() : std::vector<CoarseMatchCellStat>();
    result.bestFastScore = result.topCandidates.empty() ? 0.0 : result.topCandidates.front().fastScore;
    result.worstKeptFastScore = result.topCandidates.empty() ? 0.0 : result.topCandidates.back().fastScore;
    if (m_config.traceSuppressedCandidates) {
        appendTraceStage(result,
                         m_config,
                         "nms_suppressed_candidates",
                         m_config.enableSpatialDiversity ? diversityBuffer.suppressedCandidates() : legacyBuffer.suppressedCandidates());
    }
    appendTraceStage(result, m_config, "after_nms", result.topCandidates);
    appendTraceStage(result, m_config, "after_spatial_diversity", result.topCandidates);
    appendTraceStage(result, m_config, "level_output_beam", result.topCandidates);
    finalizeLevelStats(result);
    result.elapsedMs = elapsedMsSince(t0);
    result.votingVerifyTimeMs = result.fastScoreTimeMs + result.nmsTimeMs;
    return result;
}

CoarseMatchLevelResult CoarsePoseGenerator::searchLocal(const ShapeTemplateModel& templateLevel,
                                                        const EdgeImageData& imageLevel,
                                                        int level,
                                                        double currentLevelScale,
                                                        double previousLevelScale,
                                                        const std::vector<CoarseCandidate>& previousCandidates,
                                                        const EdgeQueryContext* queryContext,
                                                        const std::vector<cv::Rect>* activeRegions) const
{
    const auto t0 = std::chrono::steady_clock::now();
    CoarseMatchLevelResult result;
    result.level = level;
    result.imageWidth = imageLevel.imageSize.width;
    result.imageHeight = imageLevel.imageSize.height;
    result.templatePoints = templateLevel.pointCount();
    result.angleStepDeg = angleStepForLevel(level);
    result.translationStepPx = translationStepForLevel(level);
    result.inputBeamCount = static_cast<int>(previousCandidates.size());
    result.candidateSource = "local_refine";
    result.distanceFieldValid = imageLevel.hasDistanceField;
    result.scoringMode = queryContext
        ? "roi_distance_field"
        : ((m_config.enableDistanceFieldScoring && imageLevel.hasDistanceField) ? "distance_field" : "local_search");
    result.localSearchFallbackCount = (result.distanceFieldValid || queryContext) ? 0 : 1;

    TopKCandidateBuffer legacyBuffer(m_config.topKPerLevel,
                                     std::max(m_config.nmsTranslationThresholdPx, static_cast<double>(result.translationStepPx) * 2.0),
                                     std::max(m_config.nmsAngleThresholdDeg, result.angleStepDeg * 0.75));
    SpatialDiversityTopKBuffer diversityBuffer(m_config,
                                               imageLevel.imageSize.width,
                                               imageLevel.imageSize.height,
                                               m_config.topKPerLevel,
                                               std::max(m_config.nmsTranslationThresholdPx, static_cast<double>(result.translationStepPx) * 2.0),
                                               std::max(m_config.nmsAngleThresholdDeg, result.angleStepDeg * 0.75));
    CandidateBudgetPolicy budgetPolicy(m_config);
    ParentSelectionResult selected = budgetPolicy.selectParents(previousCandidates, imageLevel.imageSize, level);
    if (level <= 0 && m_config.level0BudgetV2.enableLevel0BudgetV2
        && static_cast<int>(selected.parents.size()) > m_config.level0BudgetV2.maxLevel0ParentCandidates) {
        selected.parents.resize(static_cast<size_t>(std::max(1, m_config.level0BudgetV2.maxLevel0ParentCandidates)));
        selected.afterCount = static_cast<int>(selected.parents.size());
    }
    result.parentCountBeforeSelection = selected.beforeCount;
    result.parentCountAfterSelection = selected.afterCount;
    result.level0ParentBeforeV2 = selected.beforeCount;
    result.level0ParentAfterV2 = selected.afterCount;
    if (m_config.enableStageTrace) {
        std::vector<MatchPose> inputPoses;
        inputPoses.reserve(previousCandidates.size());
        for (const CoarseCandidate& parent : previousCandidates) {
            inputPoses.push_back(convertPoseBetweenLevels(parent.pose, previousLevelScale, currentLevelScale));
        }
        appendTraceStageFromPoses(result, m_config, "level_initial_candidates", inputPoses);
        appendTraceStageFromPoses(result, m_config, "parent_before_selection", inputPoses);
        std::vector<MatchPose> selectedPoses;
        selectedPoses.reserve(selected.parents.size());
        for (const CoarseCandidate& parent : selected.parents) {
            selectedPoses.push_back(convertPoseBetweenLevels(parent.pose, previousLevelScale, currentLevelScale));
        }
        appendTraceStageFromPoses(result, m_config, "parent_after_selection", selectedPoses);
    }
    const int candidateBudget = (level <= 0 && m_config.level0BudgetV2.enableLevel0BudgetV2)
        ? std::max(1, std::min(m_config.level0BudgetV2.maxLevel0ScoredChildren, budgetPolicy.candidateBudgetForLevel(level)))
        : budgetPolicy.candidateBudgetForLevel(level);
    result.maxChildrenPerParent = level <= 0 && m_config.level0BudgetV2.enableLevel0BudgetV2
        ? std::max(1, m_config.level0BudgetV2.maxLevel0ChildrenPerParent)
        : (level <= 0 ? m_config.maxChildrenPerParentLevel0 : m_config.maxChildrenPerParentCoarse);

    std::vector<MatchPose> poses;
    std::vector<double> parentBeamScores;
    std::vector<double> parentVoteScores;
    std::vector<int> parentVoteCounts;
    std::vector<MatchPose> childTrace;
    std::vector<MatchPose> clippedTrace;
    poses.reserve(static_cast<size_t>(candidateBudget));
    parentBeamScores.reserve(static_cast<size_t>(candidateBudget));
    parentVoteScores.reserve(static_cast<size_t>(candidateBudget));
    parentVoteCounts.reserve(static_cast<size_t>(candidateBudget));
    std::unordered_set<long long> seen;
    seen.reserve(static_cast<size_t>(candidateBudget * 2));
    int generatedChildren = 0;
    for (const CoarseCandidate& parent : selected.parents) {
        const MatchPose center = convertPoseBetweenLevels(parent.pose, previousLevelScale, currentLevelScale);
        const RefineWindow window = budgetPolicy.refineWindowForParent(parent, level);
        const int maxChildrenForParent = (level <= 0 && m_config.level0BudgetV2.enableLevel0BudgetV2)
            ? std::max(1, m_config.level0BudgetV2.maxLevel0ChildrenPerParent)
            : window.maxChildren;
        const std::vector<LocalOffset> offsets = buildLocalOffsets(window, result.translationStepPx, result.angleStepDeg);
        int childrenForParent = 0;
        for (const LocalOffset& offset : offsets) {
            if (level <= 0 && m_config.level0BudgetV2.enableLevel0BudgetV2
                && result.rawGeneratedCandidates >= m_config.level0BudgetV2.maxLevel0RawChildren) {
                result.candidateLimitHit = true;
                break;
            }
            ++result.rawGeneratedCandidates;
            ++generatedChildren;
            MatchPose pose = center;
            pose.x += offset.dx;
            pose.y += offset.dy;
            pose.theta = wrapToPi(center.theta + degToRad(offset.da));
            if (!poseCenterInSearchRoi(m_config, pose, level)
                || pose.x < 0.0 || pose.y < 0.0
                || pose.x >= imageLevel.imageSize.width || pose.y >= imageLevel.imageSize.height) {
                continue;
            }
            if (level <= 0 && m_config.level0BudgetV2.enableRegionAwareBudget && !poseCenterInRegions(pose, activeRegions)) {
                ++result.level0RegionRejectedChildren;
                continue;
            }
            ++result.level0ChildrenAfterRegionFilter;
            ++result.afterPreFilterCandidates;
            if (m_config.enableStageTrace && static_cast<int>(childTrace.size()) < m_config.maxTraceCandidatesPerStage) {
                childTrace.push_back(pose);
            }
            const long long key = poseKey(pose);
            if (!seen.insert(key).second) {
                ++result.level0DuplicateRejectedChildren;
                continue;
            }
            ++result.level0ChildrenAfterDuplicateFilter;
            if (static_cast<int>(poses.size()) >= candidateBudget || childrenForParent >= maxChildrenForParent) {
                ++result.budgetClippedCount;
                if (m_config.enableStageTrace && m_config.traceBudgetClippedCandidates
                    && static_cast<int>(clippedTrace.size()) < m_config.maxTraceCandidatesPerStage) {
                    clippedTrace.push_back(pose);
                }
                continue;
            }
            poses.push_back(pose);
            parentBeamScores.push_back(candidateBeamScore(parent));
            parentVoteScores.push_back(parent.voteScore);
            parentVoteCounts.push_back(parent.voteCount);
            ++childrenForParent;
        }
        if (level <= 0 && m_config.level0BudgetV2.enableLevel0BudgetV2
            && result.rawGeneratedCandidates >= m_config.level0BudgetV2.maxLevel0RawChildren) {
            break;
        }
    }
    result.generatedCandidateCount = result.rawGeneratedCandidates;
    result.afterBudgetCandidates = static_cast<int>(poses.size());
    result.level0RawChildrenBeforeV2 = result.rawGeneratedCandidates;
    result.level0ChildrenAfterBudgetV2 = result.afterBudgetCandidates;
    result.avgChildrenPerParent = result.parentCountAfterSelection > 0
        ? static_cast<double>(generatedChildren) / static_cast<double>(result.parentCountAfterSelection)
        : 0.0;
    result.candidateLimitHit = result.budgetClippedCount > 0;
    appendTraceStageFromPoses(result, m_config, "local_children_generated", childTrace);
    appendTraceStageFromPoses(result, m_config, "after_candidate_budget", poses);
    if (!clippedTrace.empty()) {
        appendTraceStageFromPoses(result, m_config, "budget_clipped_candidates", clippedTrace, "budget_clipped");
    }

    FastPoseScorer scorer(m_config);
    const auto soaT0 = std::chrono::steady_clock::now();
    TemplatePointSoA points = TemplatePointSoABuilder().build(templateLevel, m_config.enablePointOrdering);
    result.soaBuildTimeMs = elapsedMsSince(soaT0);
    const auto cacheT0 = std::chrono::steady_clock::now();
    RotatedTemplateCache rotationCache;
    const bool cacheBuilt = m_config.enableRotatedTemplateCache
        && rotationCache.build(points, buildUniformThetaBinsRad(m_config.minAngleDeg,
                                                                m_config.maxAngleDeg,
                                                                cacheStepForLevel(m_config, level)));
    result.rotatedCacheBuildTimeMs = elapsedMsSince(cacheT0);
    result.scorerMode = (m_config.enableParallelCandidateScoring
                         && static_cast<int>(poses.size()) >= m_config.minCandidatesForParallelScoring
                         && cacheBuilt
                         && m_config.enableTemplatePointSoA
                         && (imageLevel.hasDistanceField || queryContext != nullptr))
        ? "parallel_soa_cache"
        : (cacheBuilt && m_config.enableTemplatePointSoA && (imageLevel.hasDistanceField || queryContext != nullptr) ? "soa_cache" : "old");

    if (result.scorerMode == "parallel_soa_cache") {
        ParallelCandidateScorer parallelScorer(m_config);
        ParallelScoringResult scored = queryContext
            ? parallelScorer.scoreCandidates(poses, points, rotationCache, *queryContext, level, m_config.topKPerLevel)
            : parallelScorer.scoreCandidates(poses, points, rotationCache, imageLevel, level, m_config.topKPerLevel);
        result.fastScoreTimeMs += scored.scoringTimeMs;
        result.parallelScoringTimeMs = scored.scoringTimeMs;
        result.mergeTimeMs = scored.mergeTimeMs;
        result.numScoringThreads = scored.numThreads;
        result.evaluatedCandidateCount = static_cast<int>(scored.candidates.size());
        int traceId = 0;
        for (size_t i = 0; i < scored.candidates.size(); ++i) {
            CoarseCandidate& candidate = scored.candidates[i];
            candidate.traceCandidateId = traceId++;
            candidate.source = "local_refine";
            const double parentScore = i < parentBeamScores.size() ? parentBeamScores[i] : 0.0;
            candidate.propagationScore = std::max(candidate.fastScore, parentScore * 0.85);
            candidate.voteScore = i < parentVoteScores.size() ? parentVoteScores[i] : 0.0;
            candidate.voteCount = i < parentVoteCounts.size() ? parentVoteCounts[i] : 0;
            accumulateScoreStats(result, candidate);
            if (candidateBeamScore(candidate) >= m_config.minFastScoreForPropagation) {
                if (m_config.enableSpatialDiversity) {
                    diversityBuffer.add(candidate);
                } else {
                    legacyBuffer.add(candidate);
                }
            }
        }
        appendTraceStage(result, m_config, "after_fast_scoring", scored.candidates);
        std::vector<CoarseCandidate> afterPruningTrace;
        for (const CoarseCandidate& candidate : scored.candidates) {
            if (!candidate.rejectedByEarlyExit) {
                afterPruningTrace.push_back(candidate);
            }
        }
        appendTraceStage(result, m_config, "after_upper_bound_pruning", afterPruningTrace);
    } else {
        result.numScoringThreads = 1;
        std::vector<CoarseCandidate> scoredTrace;
        scoredTrace.reserve(static_cast<size_t>(std::max(0, m_config.maxTraceCandidatesPerStage)));
        int traceId = 0;
        for (size_t i = 0; i < poses.size(); ++i) {
            const MatchPose& pose = poses[i];
            const FastScoreContext context = scoreContext(m_config, diversityBuffer, legacyBuffer);
            const auto scoreT0 = std::chrono::steady_clock::now();
            CoarseCandidate candidate = queryContext
                ? scorer.scorePoseFast(points, rotationCache, *queryContext, pose, level, context)
                : (result.scorerMode == "soa_cache")
                ? scorer.scorePoseFast(points, rotationCache, imageLevel, pose, level, context)
                : scorer.scorePose(templateLevel, imageLevel, pose, level, context);
            candidate.traceCandidateId = traceId++;
            candidate.source = "local_refine";
            const double parentScore = i < parentBeamScores.size() ? parentBeamScores[i] : 0.0;
            candidate.propagationScore = std::max(candidate.fastScore, parentScore * 0.85);
            candidate.voteScore = i < parentVoteScores.size() ? parentVoteScores[i] : 0.0;
            candidate.voteCount = i < parentVoteCounts.size() ? parentVoteCounts[i] : 0;
            result.fastScoreTimeMs += elapsedMsSince(scoreT0);
            ++result.evaluatedCandidateCount;
            accumulateScoreStats(result, candidate);
            if (m_config.enableStageTrace && static_cast<int>(scoredTrace.size()) < m_config.maxTraceCandidatesPerStage) {
                scoredTrace.push_back(candidate);
            }
            if (candidateBeamScore(candidate) >= m_config.minFastScoreForPropagation) {
                if (m_config.enableSpatialDiversity) {
                    diversityBuffer.add(candidate);
                } else {
                    legacyBuffer.add(candidate);
                }
            }
        }
        appendTraceStage(result, m_config, "after_fast_scoring", scoredTrace);
        std::vector<CoarseCandidate> afterPruningTrace;
        for (const CoarseCandidate& candidate : scoredTrace) {
            if (!candidate.rejectedByEarlyExit) {
                afterPruningTrace.push_back(candidate);
            }
        }
        appendTraceStage(result, m_config, "after_upper_bound_pruning", afterPruningTrace);
    }

    const auto nmsT0 = std::chrono::steady_clock::now();
    result.topCandidates = m_config.enableSpatialDiversity
        ? diversityBuffer.sortedCandidates()
        : legacyBuffer.sortedCandidates();
    result.nmsTimeMs += elapsedMsSince(nmsT0);
    result.keptCandidateCount = static_cast<int>(result.topCandidates.size());
    result.nmsBeforeCount = m_config.enableSpatialDiversity ? diversityBuffer.nmsBeforeCount() : result.keptCandidateCount;
    result.nmsAfterCount = m_config.enableSpatialDiversity ? diversityBuffer.nmsAfterCount() : result.keptCandidateCount;
    result.cellStats = m_config.enableSpatialDiversity ? diversityBuffer.cellStats() : std::vector<CoarseMatchCellStat>();
    result.bestFastScore = result.topCandidates.empty() ? 0.0 : result.topCandidates.front().fastScore;
    result.worstKeptFastScore = result.topCandidates.empty() ? 0.0 : result.topCandidates.back().fastScore;
    if (m_config.traceSuppressedCandidates) {
        appendTraceStage(result,
                         m_config,
                         "nms_suppressed_candidates",
                         m_config.enableSpatialDiversity ? diversityBuffer.suppressedCandidates() : legacyBuffer.suppressedCandidates());
    }
    appendTraceStage(result, m_config, "after_nms", result.topCandidates);
    appendTraceStage(result, m_config, "after_spatial_diversity", result.topCandidates);
    appendTraceStage(result, m_config, "level_output_beam", result.topCandidates);
    finalizeLevelStats(result);
    result.elapsedMs = elapsedMsSince(t0);
    return result;
}

MatchPose CoarsePoseGenerator::convertPoseBetweenLevels(const MatchPose& pose,
                                                        double fromLevelScale,
                                                        double toLevelScale)
{
    const double ratio = toLevelScale / std::max(1e-12, fromLevelScale);
    MatchPose out = pose;
    out.x *= ratio;
    out.y *= ratio;
    return out;
}

std::vector<double> CoarsePoseGenerator::scaleValues() const
{
    if (!m_config.enableScaleSearch || std::abs(m_config.maxScale - m_config.minScale) < 1e-12) {
        return {1.0};
    }
    std::vector<double> values;
    const double step = std::max(1e-6, m_config.scaleStep);
    for (double s = m_config.minScale; s <= m_config.maxScale + 1e-9; s += step) {
        values.push_back(s);
    }
    return values.empty() ? std::vector<double>{1.0} : values;
}

double CoarsePoseGenerator::angleStepForLevel(int level) const
{
    if (level <= 0) {
        return std::max(0.1, m_config.fineAngleStepDeg);
    }
    const double t = std::min(1.0, static_cast<double>(level) / std::max(1, m_config.pyramidLevels - 1));
    return std::max(0.1, m_config.fineAngleStepDeg + (m_config.coarseAngleStepDeg - m_config.fineAngleStepDeg) * t);
}

int CoarsePoseGenerator::translationStepForLevel(int level) const
{
    if (level <= 0) {
        return std::max(1, m_config.fineTranslationStepPx);
    }
    const double t = std::min(1.0, static_cast<double>(level) / std::max(1, m_config.pyramidLevels - 1));
    return std::max(1, static_cast<int>(std::round(m_config.fineTranslationStepPx
        + (m_config.coarseTranslationStepPx - m_config.fineTranslationStepPx) * t)));
}

} // namespace ShapeMatch
