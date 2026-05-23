#include "shape_match/coarse/CoarseMatchReportWriter.h"

#include <nlohmann/json.hpp>

#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

namespace ShapeMatch {

namespace {

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

nlohmann::json coarseCandidateJson(const CoarseCandidate& c)
{
    return {
        {"pose", poseJson(c.pose)},
        {"source", c.source},
        {"voteScore", c.voteScore},
        {"voteCount", c.voteCount},
        {"fastScore", c.fastScore},
        {"coverageApprox", c.coverageApprox},
        {"orientationApprox", c.orientationApprox},
        {"polarityApprox", c.polarityApprox},
        {"pyramidLevel", c.pyramidLevel},
        {"evaluatedPoints", c.evaluatedPoints},
        {"totalPointCount", c.totalPointCount},
        {"matchedPoints", c.matchedPoints},
        {"rejectedByEarlyExit", c.rejectedByEarlyExit},
        {"rejectReason", c.rejectReason}
    };
}

const CoarseCandidate* findCoarseByPose(const std::vector<CoarseCandidate>& candidates, const MatchPose& pose)
{
    const CoarseCandidate* best = nullptr;
    double bestCost = std::numeric_limits<double>::max();
    for (const CoarseCandidate& c : candidates) {
        const double dx = c.pose.x - pose.x;
        const double dy = c.pose.y - pose.y;
        const double da = std::abs(radToDeg(wrapToPi(c.pose.theta - pose.theta)));
        const double cost = std::sqrt(dx * dx + dy * dy) + 0.25 * da + std::abs(c.pose.scale - pose.scale) * 10.0;
        if (cost < bestCost) {
            bestCost = cost;
            best = &c;
        }
    }
    return best;
}

} // namespace

bool CoarseMatchReportWriter::writeAll(const CoarseMatchReport& report,
                                       const CoarseMatchConfig& config,
                                       const std::filesystem::path& reportsDir) const
{
    std::filesystem::create_directories(reportsDir);

    nlohmann::json root;
    root["config"] = config.toString();
    root["currentPreset"] = CoarseMatchConfig::presetName(config.currentPreset);
    root["totalTimeMs"] = report.totalTimeMs;
    root["failureReason"] = report.failureReason;
    root["initialCandidateMode"] = report.initialCandidateMode;
    root["votingRawCandidateCount"] = report.votingRawCandidateCount;
    root["votingScoredCandidateCount"] = report.votingScoredCandidateCount;
    root["votingBestScore"] = report.votingBestScore;
    root["votingTimeMs"] = report.votingTimeMs;
    root["gridFallbackUsed"] = report.gridFallbackUsed;
    root["multiTargetEval"] = {
        {"candidateCount", report.multiTargetEval.candidateCount},
        {"gtCount", report.multiTargetEval.gtCount},
        {"truePositive", report.multiTargetEval.truePositive},
        {"falsePositive", report.multiTargetEval.falsePositive},
        {"falseNegative", report.multiTargetEval.falseNegative},
        {"duplicateCount", report.multiTargetEval.duplicateCount},
        {"precision", report.multiTargetEval.precision},
        {"recall", report.multiTargetEval.recall},
        {"f1", report.multiTargetEval.f1},
        {"top1Hit", report.multiTargetEval.top1Hit},
        {"top3Hit", report.multiTargetEval.top3Hit},
        {"top5Hit", report.multiTargetEval.top5Hit},
        {"top10Hit", report.multiTargetEval.top10Hit}
    };
    for (const CoarseMatchLevelResult& level : report.levels) {
        nlohmann::json jLevel = {
            {"level", level.level},
            {"imageWidth", level.imageWidth},
            {"imageHeight", level.imageHeight},
            {"templatePoints", level.templatePoints},
            {"angleStepDeg", level.angleStepDeg},
            {"translationStepPx", level.translationStepPx},
            {"evaluatedCandidateCount", level.evaluatedCandidateCount},
            {"generatedCandidateCount", level.generatedCandidateCount},
            {"inputBeamCount", level.inputBeamCount},
            {"earlyExitCandidateCount", level.earlyExitCandidateCount},
            {"nmsBeforeCount", level.nmsBeforeCount},
            {"nmsAfterCount", level.nmsAfterCount},
            {"keptCandidateCount", level.keptCandidateCount},
            {"bestFastScore", level.bestFastScore},
            {"worstKeptFastScore", level.worstKeptFastScore},
            {"candidateLimitHit", level.candidateLimitHit},
            {"elapsedMs", level.elapsedMs},
            {"fastScoreTimeMs", level.fastScoreTimeMs},
            {"nmsTimeMs", level.nmsTimeMs},
            {"scoringMode", level.scoringMode},
            {"distanceFieldValid", level.distanceFieldValid},
            {"distanceFieldBuildTimeMs", level.distanceFieldBuildTimeMs},
            {"localSearchFallbackCount", level.localSearchFallbackCount},
            {"upperBoundPrunedCount", level.upperBoundPrunedCount},
            {"avgEvaluatedPointsPerCandidate", level.avgEvaluatedPointsPerCandidate},
            {"fullEvaluatedCandidateCount", level.fullEvaluatedCandidateCount},
            {"prunedCandidateCount", level.prunedCandidateCount},
            {"rawGeneratedCandidates", level.rawGeneratedCandidates},
            {"afterPreFilterCandidates", level.afterPreFilterCandidates},
            {"afterBudgetCandidates", level.afterBudgetCandidates},
            {"scoredCandidates", level.scoredCandidates},
            {"parentCountBeforeSelection", level.parentCountBeforeSelection},
            {"parentCountAfterSelection", level.parentCountAfterSelection},
            {"budgetClippedCount", level.budgetClippedCount},
            {"maxChildrenPerParent", level.maxChildrenPerParent},
            {"avgChildrenPerParent", level.avgChildrenPerParent},
            {"candidateReductionRatio", level.candidateReductionRatio},
            {"soaBuildTimeMs", level.soaBuildTimeMs},
            {"rotatedCacheBuildTimeMs", level.rotatedCacheBuildTimeMs},
            {"rotatedCacheHitRate", level.rotatedCacheHitRate},
            {"scorerMode", level.scorerMode},
            {"parallelScoringTimeMs", level.parallelScoringTimeMs},
            {"numScoringThreads", level.numScoringThreads},
            {"mergeTimeMs", level.mergeTimeMs},
            {"candidateSource", level.candidateSource},
            {"votingRawCandidateCount", level.votingRawCandidateCount},
            {"votingScoredCandidateCount", level.votingScoredCandidateCount},
            {"votingAcceptedCandidateCount", level.votingAcceptedCandidateCount},
            {"votingBestScore", level.votingBestScore},
            {"votingTimeMs", level.votingTimeMs},
            {"votingVerifyTimeMs", level.votingVerifyTimeMs},
            {"gridFallbackUsed", level.gridFallbackUsed},
            {"buildRtTableMs", level.buildRtTableMs},
            {"sampleImageEdgesMs", level.sampleImageEdgesMs},
            {"votingMs", level.votingAccumulatorMs},
            {"peakFindMs", level.peakFindMs}
        };
        for (const CoarseCandidate& c : level.topCandidates) {
            jLevel["topCandidates"].push_back(coarseCandidateJson(c));
        }
        root["levels"].push_back(std::move(jLevel));
    }
    for (const ScoredCandidate& c : report.finalRankedCandidates) {
        root["finalRankedCandidates"].push_back({
            {"rank", c.rank},
            {"pose", poseJson(c.pose)},
            {"finalScore", c.score.finalScore},
            {"coverageRatio", c.score.coverageRatio},
            {"inlierRatio", c.score.inlierRatio},
            {"rmsError", c.score.rmsError},
            {"medianError", c.score.medianError},
            {"p90Error", c.score.p90Error},
            {"poseErrorDxy", c.poseError.dxy},
            {"poseErrorDthetaDeg", c.poseError.dthetaDeg},
            {"poseOk", c.poseError.poseOk}
        });
    }

    {
        std::ofstream file(reportsDir / "latest_coarse_match_report.json");
        if (!file) {
            return false;
        }
        file << root.dump(2);
    }

    {
        std::ofstream file(reportsDir / "latest_coarse_levels.csv");
        if (!file) {
            return false;
        }
        file << "level,image_width,image_height,template_points,angle_step_deg,translation_step_px,"
                "evaluated_candidates,kept_candidates,best_fast_score,elapsed_ms,candidate_limit_hit,"
                "nms_before,nms_after,early_exit,upper_bound_pruned,avg_eval_points,fast_score_ms,nms_ms,"
                "scoring_mode,scorer_mode,distance_field_valid,distance_field_build_ms,"
                "raw_generated,after_prefilter,after_budget,scored,parent_before,parent_after,budget_clipped,"
                "candidate_reduction_ratio,soa_build_ms,rotated_cache_build_ms,parallel_scoring_ms,num_scoring_threads,merge_ms,"
                "candidate_source,"
                "voting_raw,voting_scored,voting_best,voting_time_ms,voting_verify_ms,grid_fallback\n";
        file << std::fixed << std::setprecision(6);
        for (const CoarseMatchLevelResult& level : report.levels) {
            const double best = level.topCandidates.empty() ? 0.0 : level.topCandidates.front().fastScore;
            file << level.level << ',' << level.imageWidth << ',' << level.imageHeight << ','
                 << level.templatePoints << ',' << level.angleStepDeg << ',' << level.translationStepPx << ','
                 << level.evaluatedCandidateCount << ',' << level.keptCandidateCount << ','
                 << best << ',' << level.elapsedMs << ',' << (level.candidateLimitHit ? "true" : "false") << ','
                 << level.nmsBeforeCount << ',' << level.nmsAfterCount << ',' << level.earlyExitCandidateCount << ','
                 << level.upperBoundPrunedCount << ',' << level.avgEvaluatedPointsPerCandidate << ','
                 << level.fastScoreTimeMs << ',' << level.nmsTimeMs << ',' << level.scoringMode << ','
                 << level.scorerMode << ',' << (level.distanceFieldValid ? "true" : "false") << ',' << level.distanceFieldBuildTimeMs << ','
                 << level.rawGeneratedCandidates << ',' << level.afterPreFilterCandidates << ',' << level.afterBudgetCandidates << ','
                 << level.scoredCandidates << ',' << level.parentCountBeforeSelection << ',' << level.parentCountAfterSelection << ','
                 << level.budgetClippedCount << ',' << level.candidateReductionRatio << ','
                 << level.soaBuildTimeMs << ',' << level.rotatedCacheBuildTimeMs << ',' << level.parallelScoringTimeMs << ','
                 << level.numScoringThreads << ',' << level.mergeTimeMs << ','
                 << level.candidateSource << ',' << level.votingRawCandidateCount << ','
                 << level.votingScoredCandidateCount << ',' << level.votingBestScore << ','
                 << level.votingTimeMs << ',' << level.votingVerifyTimeMs << ','
                 << (level.gridFallbackUsed ? "true" : "false") << '\n';
        }
    }

    {
        std::ofstream file(reportsDir / "latest_coarse_candidates.csv");
        if (!file) {
            return false;
        }
        file << "rank,source,level,x,y,theta_deg,scale,vote_score,vote_count,fast_score,coverage_approx,orientation_approx,polarity_approx,"
                "evaluated_points,total_points,matched_points,rejected_by_early_exit,reject_reason,final_score,coverage_ratio,"
                "inlier_ratio,rms_error,median_error,p90_error,pose_error_dxy,pose_error_dtheta_deg,pose_ok\n";
        file << std::fixed << std::setprecision(6);
        for (const ScoredCandidate& c : report.finalRankedCandidates) {
            const CoarseCandidate* coarse = findCoarseByPose(report.finalCoarseCandidates, c.pose);
            file << c.rank << ','
                 << csvEscape(coarse ? coarse->source : std::string()) << ','
                 << (coarse ? coarse->pyramidLevel : 0) << ','
                 << c.pose.x << ',' << c.pose.y << ',' << c.pose.thetaDeg() << ',' << c.pose.scale << ','
                 << (coarse ? coarse->voteScore : 0.0) << ','
                 << (coarse ? coarse->voteCount : 0) << ','
                 << (coarse ? coarse->fastScore : 0.0) << ','
                 << (coarse ? coarse->coverageApprox : 0.0) << ','
                 << (coarse ? coarse->orientationApprox : 0.0) << ','
                 << (coarse ? coarse->polarityApprox : 0.0) << ','
                 << (coarse ? coarse->evaluatedPoints : 0) << ','
                 << (coarse ? coarse->totalPointCount : 0) << ','
                 << (coarse ? coarse->matchedPoints : 0) << ','
                 << (coarse && coarse->rejectedByEarlyExit ? "true" : "false") << ','
                 << csvEscape(coarse ? coarse->rejectReason : std::string()) << ','
                 << c.score.finalScore << ',' << c.score.coverageRatio << ',' << c.score.inlierRatio << ','
                 << c.score.rmsError << ',' << c.score.medianError << ',' << c.score.p90Error << ','
                 << c.poseError.dxy << ',' << c.poseError.dthetaDeg << ','
                 << (c.poseError.poseOk ? "true" : "false") << '\n';
        }
    }

    {
        std::ofstream file(reportsDir / "latest_coarse_summary.txt");
        if (!file) {
            return false;
        }
        file << std::fixed << std::setprecision(4);
        file << "config: " << config.toString() << "\n";
        file << "currentPreset: " << CoarseMatchConfig::presetName(config.currentPreset) << "\n";
        file << "initialCandidateMode: " << report.initialCandidateMode << "\n";
        file << "votingRawCandidateCount: " << report.votingRawCandidateCount << "\n";
        file << "votingScoredCandidateCount: " << report.votingScoredCandidateCount << "\n";
        file << "votingBestScore: " << report.votingBestScore << "\n";
        file << "votingTimeMs: " << report.votingTimeMs << "\n";
        file << "gridFallbackUsed: " << (report.gridFallbackUsed ? "true" : "false") << "\n";
        file << "pyramidLevels: " << report.levels.size() << "\n";
        for (const CoarseMatchLevelResult& level : report.levels) {
            file << "level " << level.level
                 << ": evaluated=" << level.evaluatedCandidateCount
                 << " kept=" << level.keptCandidateCount
                 << " bestFast=" << (level.topCandidates.empty() ? 0.0 : level.topCandidates.front().fastScore)
                 << " elapsedMs=" << level.elapsedMs
                 << " fastScoreMs=" << level.fastScoreTimeMs
                 << " nms=" << level.nmsBeforeCount << "->" << level.nmsAfterCount
                 << " scoringMode=" << level.scoringMode
                 << " scorerMode=" << level.scorerMode
                 << " distanceFieldValid=" << (level.distanceFieldValid ? "true" : "false")
                 << " distanceFieldBuildMs=" << level.distanceFieldBuildTimeMs
                 << " rawGenerated=" << level.rawGeneratedCandidates
                 << " afterBudget=" << level.afterBudgetCandidates
                 << " parents=" << level.parentCountBeforeSelection << "->" << level.parentCountAfterSelection
                 << " budgetClipped=" << level.budgetClippedCount
                 << " threads=" << level.numScoringThreads
                 << " upperBoundPruned=" << level.upperBoundPrunedCount
                 << " avgEvalPoints=" << level.avgEvaluatedPointsPerCandidate
                 << " source=" << level.candidateSource
                 << " votingRaw=" << level.votingRawCandidateCount
                 << " votingScored=" << level.votingScoredCandidateCount
                 << " votingVerifyMs=" << level.votingVerifyTimeMs
                 << " gridFallback=" << (level.gridFallbackUsed ? "true" : "false")
                 << " limitHit=" << (level.candidateLimitHit ? "true" : "false") << "\n";
        }
        file << "finalTopK: " << report.finalRankedCandidates.size() << "\n";
        for (const ScoredCandidate& c : report.finalRankedCandidates) {
            file << "rank " << c.rank
                 << ": x=" << c.pose.x
                 << " y=" << c.pose.y
                 << " theta_deg=" << c.pose.thetaDeg()
                 << " scale=" << c.pose.scale
                 << " finalScore=" << c.score.finalScore
                 << " coverage=" << c.score.coverageRatio
                 << " inlier=" << c.score.inlierRatio
                 << " rms=" << c.score.rmsError;
            if (c.poseError.hasGroundTruth) {
                file << " dxy=" << c.poseError.dxy
                     << " dtheta_deg=" << c.poseError.dthetaDeg
                     << " poseOk=" << (c.poseError.poseOk ? "true" : "false");
            }
            file << "\n";
        }
        file << "top1Hit: " << (report.multiTargetEval.top1Hit ? "true" : "false") << "\n";
        file << "top3Hit: " << (report.multiTargetEval.top3Hit ? "true" : "false") << "\n";
        file << "top5Hit: " << (report.multiTargetEval.top5Hit ? "true" : "false") << "\n";
        file << "top10Hit: " << (report.multiTargetEval.top10Hit ? "true" : "false") << "\n";
        file << "TP/FP/FN: " << report.multiTargetEval.truePositive << '/'
             << report.multiTargetEval.falsePositive << '/' << report.multiTargetEval.falseNegative << "\n";
        file << "Precision/Recall/F1: " << report.multiTargetEval.precision << '/'
             << report.multiTargetEval.recall << '/' << report.multiTargetEval.f1 << "\n";
        file << "totalTimeMs: " << report.totalTimeMs << "\n";
        file << "profileFastScoreTimeMs: " << report.profile.fastScoreTimeMs << "\n";
        file << "profileDistanceFieldBuildTimeMs: " << report.profile.distanceFieldBuildTimeMs << "\n";
        file << "profileUpperBoundPrunedCount: " << report.profile.upperBoundPrunedCount << "\n";
        file << "profileAvgEvaluatedPointsPerCandidate: " << report.profile.avgEvaluatedPointsPerCandidate << "\n";
        file << "profileCandidateReductionRatio: " << report.profile.candidateReductionRatio << "\n";
        file << "profileLevel0CandidateCountBefore: " << report.profile.level0CandidateCountBefore << "\n";
        file << "profileLevel0CandidateCountAfterBudget: " << report.profile.level0CandidateCountAfterBudget << "\n";
        file << "profileLevel0ScoredCandidateCount: " << report.profile.level0ScoredCandidateCount << "\n";
        file << "profileSoaBuildTimeMs: " << report.profile.soaBuildTimeMs << "\n";
        file << "profileRotatedCacheBuildTimeMs: " << report.profile.rotatedCacheBuildTimeMs << "\n";
        file << "profileParallelScoringTimeMs: " << report.profile.parallelScoringTimeMs << "\n";
        file << "profileMergeTimeMs: " << report.profile.mergeTimeMs << "\n";
        file << "reached100ms: " << (report.totalTimeMs <= 100.0 ? "true" : "false") << "\n";
        file << "profileNmsTimeMs: " << report.profile.nmsTimeMs << "\n";
        file << "profileFinalRankerTimeMs: " << report.profile.finalRankerTimeMs << "\n";
        file << "largeImageMode: " << (report.profile.largeImageMode ? "true" : "false") << "\n";
        file << "skippedFullLevel0DistanceField: " << (report.profile.skippedFullLevel0DistanceField ? "true" : "false") << "\n";
        file << "roiDistanceFieldCount: " << report.profile.roiDistanceFieldCount << "\n";
        file << "roiDistanceFieldTotalPixels: " << report.profile.roiDistanceFieldTotalPixels << "\n";
        file << "roiDistanceFieldBuildTimeMs: " << report.profile.roiDistanceFieldBuildTimeMs << "\n";
        file << "level0ParentBefore: " << report.profile.level0ParentBefore << "\n";
        file << "level0ParentAfter: " << report.profile.level0ParentAfter << "\n";
        file << "level0RawChildren: " << report.profile.level0RawChildren << "\n";
        file << "level0ScoredChildren: " << report.profile.level0ScoredChildren << "\n";
        file << "finalRankerTimeMs: " << report.profile.finalRankerTimeMs << "\n";
        file << "finalRankerPercent: "
             << (report.totalTimeMs > 1e-9 ? report.profile.finalRankerTimeMs * 100.0 / report.totalTimeMs : 0.0)
             << "\n";
        file << "finalRankerEvaluationMode: " << report.profile.finalRankerProfile.evaluationMode << "\n";
        file << "finalRankerCandidateCountBeforeNms: "
             << report.profile.finalRankerProfile.candidateCountBeforeNms << "\n";
        file << "finalRankerCandidateCountAfterNms: "
             << report.profile.finalRankerProfile.candidateCountAfterNms << "\n";
        file << "finalRankerPointEvalMode: "
             << (report.profile.finalRankerProfile.pointEvalEnabled ? "top_k_debug" : "disabled") << "\n";
        file << "finalRankerPointEvalTopK: " << report.profile.finalRankerProfile.pointEvalTopK << "\n";
        file << "finalRankerPointEvalTimeMs: " << report.profile.finalRankerProfile.pointEvalTimeMs << "\n";
        file << "finalRankerLinearScanFallbackCount: "
             << report.profile.finalRankerProfile.linearScanFallbackCount << "\n";
        file << "finalRankerRoiDistanceFieldQueryCount: "
             << report.profile.finalRankerProfile.roiDistanceFieldQueryCount << "\n";
        file << "finalRankerFullDistanceFieldQueryCount: "
             << report.profile.finalRankerProfile.fullDistanceFieldQueryCount << "\n";
        file << "finalRankerMissingFieldCount: "
             << report.profile.finalRankerProfile.missingFieldCount << "\n";
        file << "finalRankerLocalWindowFallbackCount: "
             << report.profile.finalRankerProfile.localWindowFallbackCount << "\n";
        file << "finalRankerNumThreads: " << report.profile.finalRankerProfile.numThreads << "\n";
        if (!report.failureReason.empty()) {
            file << "failureReason: " << report.failureReason << "\n";
        }
    }

    return true;
}

} // namespace ShapeMatch
