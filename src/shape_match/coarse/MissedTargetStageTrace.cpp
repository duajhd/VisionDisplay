#include "shape_match/coarse/MissedTargetStageTrace.h"

#include "shape_match/coarse/CoarseCandidate.h"
#include "shape_match/coarse/CoarseMatchConfig.h"
#include "shape_match/coarse/CoarsePoseGenerator.h"
#include "shape_match/coarse/FastPoseScorer.h"
#include "shape_match/coarse/ImageEdgeSampler.h"
#include "shape_match/coarse/ImagePyramid.h"
#include "shape_match/coarse/TemplatePyramid.h"
#include "shape_match/coarse/VotingConfig.h"
#include "shape_match/evaluation/PoseErrorEvaluator.h"
#include "shape_match/evaluation/ShapeMatchEvalConfig.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

namespace ShapeMatch {

namespace {

double elapsedMsSince(std::chrono::steady_clock::time_point t0)
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

std::string csvEscape(const std::string& text)
{
    if (text.find_first_of(",\"\n\r") == std::string::npos) {
        return text;
    }
    std::string out = "\"";
    for (char c : text) {
        out += c == '"' ? "\"\"" : std::string(1, c);
    }
    out += '"';
    return out;
}

nlohmann::json poseJson(const MatchPose& p)
{
    return {{"x", p.x}, {"y", p.y}, {"theta_deg", p.thetaDeg()}, {"scale", p.scale}};
}

double poseDxy(const MatchPose& a, const MatchPose& b)
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

double poseDthetaDeg(const MatchPose& a, const MatchPose& b)
{
    return std::abs(radToDeg(wrapToPi(a.theta - b.theta)));
}

CandidateTraceInfo traceFromCandidate(const CoarseCandidate& c,
                                      const std::string& stage,
                                      int level,
                                      int rank,
                                      const MatchPose* target = nullptr)
{
    CandidateTraceInfo info;
    info.candidateId = c.traceCandidateId;
    info.source = stage;
    info.level = level;
    info.rank = rank;
    info.pose = c.pose;
    info.voteScore = c.voteScore;
    info.voteCount = c.voteCount;
    info.fastScore = c.fastScore;
    info.coverageApprox = c.coverageApprox;
    info.orientationApprox = c.orientationApprox;
    info.polarityApprox = c.polarityApprox;
    info.exists = true;
    info.kept = true;
    info.suppressedByNms = c.suppressedByNms;
    info.suppressorCandidateId = c.suppressorCandidateId;
    info.prunedByUpperBound = c.rejectedByEarlyExit && c.rejectReason == "upper_bound_pruned";
    info.rejectReason = c.rejectReason;
    info.gridCellRow = c.sourceCellRow;
    info.gridCellCol = c.sourceCellCol;
    if (target) {
        info.dxy = poseDxy(c.pose, *target);
        info.dthetaDeg = poseDthetaDeg(c.pose, *target);
    }
    return info;
}

CandidateTraceInfo traceFromScoredCandidate(const ScoredCandidate& c,
                                            int rank,
                                            const MatchPose* target = nullptr)
{
    CandidateTraceInfo info;
    info.candidateId = rank;
    info.source = "final_ranked_candidates";
    info.level = 0;
    info.rank = rank;
    info.pose = c.pose;
    info.finalScore = c.score.finalScore;
    info.exists = true;
    info.kept = true;
    if (target) {
        info.dxy = poseDxy(c.pose, *target);
        info.dthetaDeg = poseDthetaDeg(c.pose, *target);
    }
    return info;
}

StageTraceSnapshot makeSnapshot(const std::string& stage,
                                int level,
                                const std::vector<CoarseCandidate>& candidates,
                                int maxCount)
{
    StageTraceSnapshot snapshot;
    snapshot.stage = stage;
    snapshot.level = level;
    const int count = std::min(std::max(0, maxCount), static_cast<int>(candidates.size()));
    snapshot.candidates.reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i) {
        snapshot.candidates.push_back(traceFromCandidate(candidates[static_cast<size_t>(i)], stage, level, i));
    }
    return snapshot;
}

bool isNear(const CandidateTraceInfo& info, double maxDxy, double maxDtheta)
{
    return info.exists && info.dxy <= maxDxy && info.dthetaDeg <= maxDtheta;
}

bool isPipelinePresenceStage(const std::string& stage)
{
    return stage != "nms_suppressed_candidates"
        && stage != "budget_clipped_candidates"
        && stage != "pruned_candidates";
}

CandidateTraceInfo nearestInStage(const StageTraceSnapshot& stage,
                                  const MatchPose& target,
                                  double nearDxy,
                                  double nearTheta,
                                  int* nearCount = nullptr,
                                  double* bestFastScoreNear = nullptr,
                                  double* bestFinalScoreNear = nullptr)
{
    CandidateTraceInfo best;
    double bestCost = std::numeric_limits<double>::max();
    int count = 0;
    double bestFast = 0.0;
    double bestFinal = 0.0;
    for (const CandidateTraceInfo& raw : stage.candidates) {
        CandidateTraceInfo info = raw;
        info.dxy = poseDxy(info.pose, target);
        info.dthetaDeg = poseDthetaDeg(info.pose, target);
        const bool near = isNear(info, nearDxy, nearTheta);
        if (near) {
            ++count;
            bestFast = std::max(bestFast, info.fastScore);
            bestFinal = std::max(bestFinal, info.finalScore);
        }
        const double cost = info.dxy + 0.35 * info.dthetaDeg;
        if (cost < bestCost) {
            bestCost = cost;
            best = info;
        }
    }
    if (nearCount) {
        *nearCount = count;
    }
    if (bestFastScoreNear) {
        *bestFastScoreNear = bestFast;
    }
    if (bestFinalScoreNear) {
        *bestFinalScoreNear = bestFinal;
    }
    return best;
}

bool finalHit(const std::vector<ScoredCandidate>& candidates, const MatchPose& gt)
{
    ShapeMatchEvalConfig config;
    PoseErrorEvaluator evaluator(config);
    for (const ScoredCandidate& c : candidates) {
        if (evaluator.evaluatePoseError(c.pose, gt).poseOk) {
            return true;
        }
    }
    return false;
}

std::string reasonBetween(const std::string& previousStage, const std::string& currentStage)
{
    if (currentStage == "after_candidate_budget") {
        return "budget_clipped";
    }
    if (currentStage == "parent_after_selection") {
        return "parent_selection_dropped";
    }
    if (currentStage == "after_upper_bound_pruning") {
        return "upper_bound_pruned";
    }
    if (currentStage == "after_nms") {
        return "nms_suppressed";
    }
    if (currentStage == "level_initial_candidates" && previousStage == "level_output_beam") {
        return "beam_propagation_lost";
    }
    if (currentStage == "final_ranked_candidates") {
        return "final_ranker_dropped";
    }
    if (currentStage == "voting_verified" || currentStage == "level_initial_candidates") {
        return "candidate_generation_failed";
    }
    return "unknown";
}

std::string suggestedAction(const std::string& reason)
{
    if (reason == "budget_clipped") {
        return "increase maxCandidatesLevel0/maxCandidatesLocalRefine or topKPerGridCell, or use Recall preset";
    }
    if (reason == "nms_suppressed") {
        return "reduce nmsTranslationThresholdPx/nmsAngleThresholdDeg or increase spatial diversity limits";
    }
    if (reason == "upper_bound_pruned") {
        return "reduce upperBoundMargin, disable greedy upper-bound pruning for diagnosis, or increase minEvaluatedPointsBeforePruning";
    }
    if (reason == "parent_selection_dropped") {
        return "increase maxParentsForLevel0Refine/maxParentsPerGridCell or disable parent diversity before refine";
    }
    if (reason == "beam_propagation_lost") {
        return "increase beamWidth/topKPerLevel and preserve more diverse candidates between pyramid levels";
    }
    if (reason == "image_edge_sampler_dropped_roi_points") {
        return "increase maxImagePointsPerTile/maxImageVotePoints or adjust image tiling";
    }
    if (reason == "rt_voting_not_accumulating") {
        return "inspect RT table bins, orientation binning, and vote peak thresholds";
    }
    if (reason == "final_ranker_dropped") {
        return "inspect MatchEvaluator thresholds and finalTopK/ranker ordering";
    }
    if (reason == "fast_score_model_unfriendly") {
        return "inspect template point normals/polarity and fast scorer weights";
    }
    return "compare nearest candidates in the first missing stage with the previous stage";
}

cv::Rect2d estimatedRoi(const ShapeTemplateModel& model, const MatchPose& pose)
{
    if (model.boundingBox.empty()) {
        return cv::Rect2d(pose.x - 16.0, pose.y - 16.0, 32.0, 32.0);
    }
    const std::vector<cv::Point2d> corners{
        model.boundingBox.tl(),
        cv::Point2d(model.boundingBox.x + model.boundingBox.width, model.boundingBox.y),
        cv::Point2d(model.boundingBox.x + model.boundingBox.width, model.boundingBox.y + model.boundingBox.height),
        cv::Point2d(model.boundingBox.x, model.boundingBox.y + model.boundingBox.height)
    };
    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double maxX = -std::numeric_limits<double>::max();
    double maxY = -std::numeric_limits<double>::max();
    for (const cv::Point2d& p : corners) {
        const cv::Point2d q = pose.transformPoint(p);
        minX = std::min(minX, q.x);
        minY = std::min(minY, q.y);
        maxX = std::max(maxX, q.x);
        maxY = std::max(maxY, q.y);
    }
    const double pad = 8.0;
    return cv::Rect2d(minX - pad, minY - pad, std::max(1.0, maxX - minX + 2.0 * pad), std::max(1.0, maxY - minY + 2.0 * pad));
}

bool pointInRoi(double x, double y, const cv::Rect2d& roi)
{
    return x >= roi.x && y >= roi.y && x < roi.x + roi.width && y < roi.y + roi.height;
}

float sampleFloat(const cv::Mat& mat, int x, int y, float fallback = 0.0f)
{
    if (mat.empty() || x < 0 || y < 0 || x >= mat.cols || y >= mat.rows) {
        return fallback;
    }
    return mat.at<float>(y, x);
}

RoiVoteTraceStats computeRoiVoteStats(const EdgeImageData& edgeData,
                                      const cv::Rect2d& roi,
                                      const VotingConfig& votingConfig,
                                      const std::vector<StageTraceSnapshot>& stages,
                                      const MatchPose& target)
{
    RoiVoteTraceStats stats;
    stats.gradientBinHistogram.assign(static_cast<size_t>(std::max(1, votingConfig.orientationBinCount)), 0);
    double gradSum = 0.0;
    auto addGrad = [&](int x, int y) {
        const double mag = sampleFloat(edgeData.gradMag, x, y, 1.0f);
        gradSum += mag;
        stats.maxGradMag = std::max(stats.maxGradMag, mag);
        if (!edgeData.gradX.empty() && !edgeData.gradY.empty()) {
            const double gx = sampleFloat(edgeData.gradX, x, y, 1.0f);
            const double gy = sampleFloat(edgeData.gradY, x, y, 0.0f);
            double angle = wrapToPi(std::atan2(gy, gx));
            if (angle < 0.0) {
                angle += 2.0 * kPi;
            }
            const int bin = std::clamp(static_cast<int>(std::floor(angle * stats.gradientBinHistogram.size() / (2.0 * kPi))),
                                       0,
                                       static_cast<int>(stats.gradientBinHistogram.size()) - 1);
            ++stats.gradientBinHistogram[static_cast<size_t>(bin)];
        }
    };

    if (!edgeData.edgePoints.empty()) {
        for (const cv::Point2d& p : edgeData.edgePoints) {
            if (!pointInRoi(p.x, p.y, roi)) {
                continue;
            }
            ++stats.rawEdgeCountInRoi;
            addGrad(std::clamp(static_cast<int>(std::round(p.x)), 0, std::max(0, edgeData.imageSize.width - 1)),
                    std::clamp(static_cast<int>(std::round(p.y)), 0, std::max(0, edgeData.imageSize.height - 1)));
        }
    } else if (!edgeData.edgeMap.empty()) {
        const int x0 = std::max(0, static_cast<int>(std::floor(roi.x)));
        const int y0 = std::max(0, static_cast<int>(std::floor(roi.y)));
        const int x1 = std::min(edgeData.edgeMap.cols, static_cast<int>(std::ceil(roi.x + roi.width)));
        const int y1 = std::min(edgeData.edgeMap.rows, static_cast<int>(std::ceil(roi.y + roi.height)));
        for (int y = y0; y < y1; ++y) {
            const uchar* row = edgeData.edgeMap.ptr<uchar>(y);
            for (int x = x0; x < x1; ++x) {
                if (row[x] == 0) {
                    continue;
                }
                ++stats.rawEdgeCountInRoi;
                addGrad(x, y);
            }
        }
    }

    if (stats.rawEdgeCountInRoi > 0) {
        stats.avgGradMag = gradSum / static_cast<double>(stats.rawEdgeCountInRoi);
    }

    ImageEdgeSampler sampler;
    const std::vector<ImageVotePoint> sampled = sampler.sample(edgeData, votingConfig);
    for (const ImageVotePoint& p : sampled) {
        if (pointInRoi(p.x, p.y, roi)) {
            ++stats.sampledVotePointCountInRoi;
        }
    }
    stats.votePointKeepRatioInRoi = stats.rawEdgeCountInRoi > 0
        ? static_cast<double>(stats.sampledVotePointCountInRoi) / static_cast<double>(stats.rawEdgeCountInRoi)
        : 0.0;

    bool havePeak = false;
    double nearestCost = std::numeric_limits<double>::max();
    for (const StageTraceSnapshot& stage : stages) {
        if (stage.stage != "voting_raw_peaks") {
            continue;
        }
        for (const CandidateTraceInfo& c : stage.candidates) {
            const double dxy = poseDxy(c.pose, target);
            const double dtheta = poseDthetaDeg(c.pose, target);
            const double cost = dxy + 0.35 * dtheta;
            if (cost < nearestCost) {
                nearestCost = cost;
                havePeak = true;
                stats.roiNearestVotingPeakDxy = dxy;
                stats.roiNearestVotingPeakDtheta = dtheta;
                stats.roiVoteContribution = c.voteScore;
            }
        }
    }
    if (!havePeak) {
        stats.roiNearestVotingPeakDxy = 0.0;
        stats.roiNearestVotingPeakDtheta = 0.0;
    }
    return stats;
}

std::vector<StageTraceSnapshot> collectStages(const CoarseMatchReport& report, const CoarseMatchConfig& config)
{
    std::vector<StageTraceSnapshot> stages;
    for (const CoarseMatchLevelResult& level : report.levels) {
        stages.insert(stages.end(), level.stageTrace.begin(), level.stageTrace.end());
        if (!level.topCandidates.empty()) {
            stages.push_back(makeSnapshot("level_output_beam", level.level, level.topCandidates, config.maxTraceCandidatesPerStage));
        }
    }
    if (!report.finalCoarseCandidates.empty()) {
        stages.push_back(makeSnapshot("final_coarse_poses", 0, report.finalCoarseCandidates, config.maxTraceCandidatesPerStage));
    }
    if (!report.finalRankedCandidates.empty()) {
        StageTraceSnapshot ranked;
        ranked.stage = "final_ranked_candidates";
        ranked.level = 0;
        const int count = std::min<int>(config.maxTraceCandidatesPerStage, static_cast<int>(report.finalRankedCandidates.size()));
        for (int i = 0; i < count; ++i) {
            ranked.candidates.push_back(traceFromScoredCandidate(report.finalRankedCandidates[static_cast<size_t>(i)], i));
        }
        stages.push_back(std::move(ranked));
    }
    return stages;
}

MissedTargetStageTraceReport buildTraceReport(const CoarseMatchReport& report,
                                              const CoarseMatchConfig& config,
                                              const ImagePyramid& imagePyramid,
                                              const TemplatePyramid& templatePyramid)
{
    const auto t0 = std::chrono::steady_clock::now();
    MissedTargetStageTraceReport trace;
    trace.nearDxyThresholdPx = config.traceNearDxyThresholdPx;
    trace.nearAngleThresholdDeg = config.traceNearAngleThresholdDeg;
    const std::vector<StageTraceSnapshot> stages = collectStages(report, config);
    FastPoseScorer scorer(config);

    VotingConfig votingConfig;
    votingConfig.enableOrientationVoting = true;
    votingConfig.topVotePeaks = std::max(1, config.maxVotingCandidatesToScore);

    for (const GroundTruthInstance& gt : report.groundTruthInstances) {
        MissedTargetStageInfo target;
        target.targetId = gt.id.empty() ? "target_" + std::to_string(trace.targets.size()) : gt.id;
        target.expectedPose = gt.pose;
        target.debugRoi = estimatedRoi(templatePyramid.level(0), gt.pose);
        target.hitFinalTopK = finalHit(report.finalRankedCandidates, gt.pose);
        target.hitFinalRanked = target.hitFinalTopK;

        if (imagePyramid.levelCount() > 0 && templatePyramid.levelCount() > 0) {
            const CoarseCandidate oracle0 = scorer.scorePose(templatePyramid.level(0), imagePyramid.level(0), gt.pose, 0, -1.0);
            target.oracleFastScoreLevel0 = oracle0.fastScore;
            target.oracleCoverageApproxLevel0 = oracle0.coverageApprox;
            target.oracleOrientationApproxLevel0 = oracle0.orientationApprox;
            target.oraclePolarityApproxLevel0 = oracle0.polarityApprox;
            target.oracleMatchedPointsLevel0 = oracle0.matchedPoints;
            target.oracleEvaluatedPointsLevel0 = oracle0.evaluatedPoints;
            const int topLevel = imagePyramid.levelCount() - 1;
            const MatchPose topPose = CoarsePoseGenerator::convertPoseBetweenLevels(gt.pose, 1.0, imagePyramid.scaleOfLevel(topLevel));
            const CoarseCandidate oracleTop = scorer.scorePose(templatePyramid.level(topLevel),
                                                               imagePyramid.level(topLevel),
                                                               topPose,
                                                               topLevel,
                                                               -1.0);
            target.oracleFastScoreTopLevel = oracleTop.fastScore;
        }

        bool previousNear = false;
        std::string previousStage;
        for (const StageTraceSnapshot& stage : stages) {
            MatchPose stagePose = gt.pose;
            if (stage.level > 0 && stage.level < imagePyramid.levelCount()) {
                stagePose = CoarsePoseGenerator::convertPoseBetweenLevels(gt.pose, 1.0, imagePyramid.scaleOfLevel(stage.level));
            }
            int nearCount = 0;
            double bestFastNear = 0.0;
            double bestFinalNear = 0.0;
            CandidateTraceInfo nearest = nearestInStage(stage,
                                                        stagePose,
                                                        config.traceNearDxyThresholdPx,
                                                        config.traceNearAngleThresholdDeg,
                                                        &nearCount,
                                                        &bestFastNear,
                                                        &bestFinalNear);
            nearest.source = stage.stage;
            nearest.level = stage.level;
            nearest.fastScore = std::max(nearest.fastScore, bestFastNear);
            nearest.finalScore = std::max(nearest.finalScore, bestFinalNear);
            target.nearestByStage.push_back(nearest);
            if (stage.stage == "voting_raw_peaks") {
                target.votingPeakCountNearTarget += nearCount;
            }
            if (stage.level >= 0 && stage.level < 8) {
                target.candidateCountNearTargetByLevel[stage.level] += nearCount;
            }
            if (!isPipelinePresenceStage(stage.stage)) {
                continue;
            }
            const bool currentNear = nearCount > 0;
            if (previousNear && !currentNear && target.firstMissingStage.empty()) {
                target.firstMissingStage = stage.stage;
                target.likelyReason = reasonBetween(previousStage, stage.stage);
            }
            previousNear = currentNear;
            previousStage = stage.stage;
        }

        if (config.traceRoiVoteStats && imagePyramid.levelCount() > 0) {
            target.roiVoteStats = computeRoiVoteStats(imagePyramid.level(0), target.debugRoi, votingConfig, stages, gt.pose);
            target.rawEdgeCountInRoi = target.roiVoteStats.rawEdgeCountInRoi;
            target.sampledVotePointCountInRoi = target.roiVoteStats.sampledVotePointCountInRoi;
        }

        if (target.firstMissingStage.empty() && !target.hitFinalTopK) {
            target.firstMissingStage = "final_ranked_candidates";
            target.likelyReason = "final_ranker_dropped";
        }
        if (target.oracleFastScoreLevel0 < 0.10) {
            target.likelyReason = "fast_score_model_unfriendly";
        } else if (target.rawEdgeCountInRoi > 0
                   && target.roiVoteStats.votePointKeepRatioInRoi < 0.10
                   && target.rawEdgeCountInRoi >= 20) {
            target.likelyReason = "image_edge_sampler_dropped_roi_points";
        } else if (target.sampledVotePointCountInRoi > 0 && target.votingPeakCountNearTarget == 0) {
            target.likelyReason = "rt_voting_not_accumulating";
        } else if (target.likelyReason.empty() && !target.hitFinalTopK) {
            target.likelyReason = "candidate_generation_failed";
        }

        std::ostringstream diagnosis;
        diagnosis << "Target " << target.targetId;
        if (target.hitFinalTopK) {
            diagnosis << " reached final topK.";
        } else {
            diagnosis << " missed at " << target.firstMissingStage << " likelyReason=" << target.likelyReason
                      << ". Suggested action: " << suggestedAction(target.likelyReason) << ".";
        }
        target.diagnosis = diagnosis.str();
        trace.targets.push_back(std::move(target));
    }

    trace.totalTraceTimeMs = elapsedMsSince(t0);
    return trace;
}

nlohmann::json traceCandidateJson(const CandidateTraceInfo& c)
{
    return {
        {"candidate_id", c.candidateId},
        {"source", c.source},
        {"level", c.level},
        {"rank", c.rank},
        {"pose", poseJson(c.pose)},
        {"nearest_dxy", c.dxy},
        {"nearest_dtheta_deg", c.dthetaDeg},
        {"vote_score", c.voteScore},
        {"vote_count", c.voteCount},
        {"fast_score", c.fastScore},
        {"final_score", c.finalScore},
        {"coverage_approx", c.coverageApprox},
        {"orientation_approx", c.orientationApprox},
        {"polarity_approx", c.polarityApprox},
        {"exists", c.exists},
        {"kept", c.kept},
        {"suppressed_by_nms", c.suppressedByNms},
        {"clipped_by_budget", c.clippedByBudget},
        {"pruned_by_upper_bound", c.prunedByUpperBound},
        {"dropped_by_parent_selection", c.droppedByParentSelection},
        {"dropped_by_parallel_merge", c.droppedByParallelMerge},
        {"reject_reason", c.rejectReason},
        {"suppressor_candidate_id", c.suppressorCandidateId},
        {"grid_cell_row", c.gridCellRow},
        {"grid_cell_col", c.gridCellCol}
    };
}

} // namespace

bool MissedTargetStageTraceWriter::writeAll(const CoarseMatchReport& report,
                                            const CoarseMatchConfig& config,
                                            const ImagePyramid& imagePyramid,
                                            const TemplatePyramid& templatePyramid,
                                            const std::filesystem::path& reportsDir) const
{
    if (report.groundTruthInstances.empty()) {
        return true;
    }
    const MissedTargetStageTraceReport trace = buildTraceReport(report, config, imagePyramid, templatePyramid);
    return writeAll(trace, reportsDir);
}

bool MissedTargetStageTraceWriter::writeAll(const MissedTargetStageTraceReport& trace,
                                            const std::filesystem::path& reportsDir) const
{
    std::filesystem::create_directories(reportsDir);
    nlohmann::json root;
    root["image"] = trace.imageName;
    root["template"] = trace.templateName;
    root["near_dxy_threshold_px"] = trace.nearDxyThresholdPx;
    root["near_angle_threshold_deg"] = trace.nearAngleThresholdDeg;
    root["total_trace_time_ms"] = trace.totalTraceTimeMs;

    for (const MissedTargetStageInfo& target : trace.targets) {
        nlohmann::json jt;
        jt["target_id"] = target.targetId;
        jt["expected_pose"] = poseJson(target.expectedPose);
        jt["debug_roi"] = {
            {"x", target.debugRoi.x},
            {"y", target.debugRoi.y},
            {"width", target.debugRoi.width},
            {"height", target.debugRoi.height}
        };
        jt["hit_final_topk"] = target.hitFinalTopK;
        jt["hit_final_ranked"] = target.hitFinalRanked;
        jt["first_missing_stage"] = target.firstMissingStage;
        jt["likely_reason"] = target.likelyReason;
        jt["oracle_fast_score_level0"] = target.oracleFastScoreLevel0;
        jt["oracle_fast_score_top_level"] = target.oracleFastScoreTopLevel;
        jt["oracle_coverage_approx_level0"] = target.oracleCoverageApproxLevel0;
        jt["oracle_orientation_approx_level0"] = target.oracleOrientationApproxLevel0;
        jt["oracle_polarity_approx_level0"] = target.oraclePolarityApproxLevel0;
        jt["oracle_matched_points_level0"] = target.oracleMatchedPointsLevel0;
        jt["oracle_evaluated_points_level0"] = target.oracleEvaluatedPointsLevel0;
        jt["raw_edge_count_in_roi"] = target.rawEdgeCountInRoi;
        jt["sampled_vote_point_count_in_roi"] = target.sampledVotePointCountInRoi;
        jt["vote_point_keep_ratio_in_roi"] = target.roiVoteStats.votePointKeepRatioInRoi;
        jt["roi_vote_contribution"] = target.roiVoteStats.roiVoteContribution;
        jt["roi_nearest_voting_peak_dxy"] = target.roiVoteStats.roiNearestVotingPeakDxy;
        jt["roi_nearest_voting_peak_dtheta"] = target.roiVoteStats.roiNearestVotingPeakDtheta;
        jt["avg_grad_mag_in_roi"] = target.roiVoteStats.avgGradMag;
        jt["max_grad_mag_in_roi"] = target.roiVoteStats.maxGradMag;
        jt["gradient_bin_histogram"] = target.roiVoteStats.gradientBinHistogram;
        jt["voting_peak_count_near_target"] = target.votingPeakCountNearTarget;
        jt["diagnosis"] = target.diagnosis;
        for (const CandidateTraceInfo& stage : target.nearestByStage) {
            jt["stages"].push_back(traceCandidateJson(stage));
        }
        root["targets"].push_back(std::move(jt));
    }

    {
        std::ofstream file(reportsDir / "latest_missed_target_stage_trace.json");
        if (!file) {
            return false;
        }
        file << root.dump(2);
    }

    {
        std::ofstream file(reportsDir / "latest_missed_target_stage_trace.csv");
        if (!file) {
            return false;
        }
        file << "target_id,stage,exists_near_target,nearest_dxy,nearest_dtheta_deg,near_candidate_count,"
                "best_fast_score_near_target,best_final_score_near_target,oracle_fast_score,dropped_reason,"
                "suppressor_candidate_id,grid_cell,level,rank\n";
        file << std::fixed << std::setprecision(6);
        for (const MissedTargetStageInfo& target : trace.targets) {
            for (const CandidateTraceInfo& stage : target.nearestByStage) {
                const bool existsNear = stage.exists
                    && stage.dxy <= trace.nearDxyThresholdPx
                    && stage.dthetaDeg <= trace.nearAngleThresholdDeg;
                const std::string gridCell = stage.gridCellRow >= 0 && stage.gridCellCol >= 0
                    ? std::to_string(stage.gridCellRow) + ":" + std::to_string(stage.gridCellCol)
                    : "";
                file << csvEscape(target.targetId) << ','
                     << csvEscape(stage.source) << ','
                     << (existsNear ? "true" : "false") << ','
                     << stage.dxy << ','
                     << stage.dthetaDeg << ','
                     << (existsNear ? 1 : 0) << ','
                     << stage.fastScore << ','
                     << stage.finalScore << ','
                     << target.oracleFastScoreLevel0 << ','
                     << csvEscape(stage.rejectReason.empty() ? target.likelyReason : stage.rejectReason) << ','
                     << stage.suppressorCandidateId << ','
                     << csvEscape(gridCell) << ','
                     << stage.level << ','
                     << stage.rank << '\n';
            }
        }
    }

    {
        std::ofstream file(reportsDir / "latest_missed_target_stage_trace.txt");
        if (!file) {
            return false;
        }
        file << std::fixed << std::setprecision(4);
        for (const MissedTargetStageInfo& target : trace.targets) {
            file << "Target " << target.targetId << ":\n";
            file << "- Final hit: " << (target.hitFinalTopK ? "true" : "false") << "\n";
            file << "- Oracle fast score level0: " << target.oracleFastScoreLevel0 << "\n";
            file << "- Oracle fast score top level: " << target.oracleFastScoreTopLevel << "\n";
            file << "- ROI raw edge points: " << target.rawEdgeCountInRoi << "\n";
            file << "- ROI sampled vote points: " << target.sampledVotePointCountInRoi << "\n";
            file << "- ROI sampled keep ratio: " << target.roiVoteStats.votePointKeepRatioInRoi << "\n";
            file << "- First missing stage: " << target.firstMissingStage << "\n";
            file << "- Likely reason: " << target.likelyReason << "\n";
            file << "- Suggested action: " << suggestedAction(target.likelyReason) << "\n";
            file << "- Diagnosis: " << target.diagnosis << "\n";
            for (const CandidateTraceInfo& stage : target.nearestByStage) {
                file << "  stage " << stage.source
                     << " level=" << stage.level
                     << " rank=" << stage.rank
                     << " nearestDxy=" << stage.dxy
                     << " nearestDthetaDeg=" << stage.dthetaDeg
                     << " fastScore=" << stage.fastScore
                     << " finalScore=" << stage.finalScore
                     << " rejectReason=" << stage.rejectReason << "\n";
            }
            file << "\n";
        }
    }

    return true;
}

ShapeMatchOverlayData buildMissedTargetStageTraceOverlay(const MissedTargetStageTraceReport& trace)
{
    ShapeMatchOverlayData overlay;
    for (const MissedTargetStageInfo& target : trace.targets) {
        const std::string color = target.hitFinalTopK ? "#22c55e" : "#ff4d4d";
        const cv::Rect2d& roi = target.debugRoi;
        overlay.polylines.push_back({
            {
                cv::Point2d(roi.x, roi.y),
                cv::Point2d(roi.x + roi.width, roi.y),
                cv::Point2d(roi.x + roi.width, roi.y + roi.height),
                cv::Point2d(roi.x, roi.y + roi.height),
                cv::Point2d(roi.x, roi.y)
            },
            color,
            2.0,
            target.targetId
        });
        overlay.texts.push_back({
            cv::Point2d(roi.x, roi.y - 8.0),
            target.hitFinalTopK ? target.targetId + " final hit" : target.targetId + " " + target.likelyReason,
            color,
            13
        });
        for (const CandidateTraceInfo& stage : target.nearestByStage) {
            if (!stage.exists) {
                continue;
            }
            const bool near = stage.dxy <= trace.nearDxyThresholdPx
                && stage.dthetaDeg <= trace.nearAngleThresholdDeg;
            overlay.points.push_back({
                cv::Point2d(stage.pose.x, stage.pose.y),
                near ? "#5ac8fa" : "#ffb020",
                near ? 4.0 : 3.0,
                stage.source
            });
        }
    }
    return overlay;
}

} // namespace ShapeMatch
