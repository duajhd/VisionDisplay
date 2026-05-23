#include "shape_match/coarse/CoarseMatchProfiler.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace ShapeMatch {

namespace {

struct TimingRow
{
    std::string name;
    double timeMs = 0.0;
    std::string detail;
};

double percentOf(double value, double total)
{
    return total > 1e-9 ? value * 100.0 / total : 0.0;
}

double safeDiv(double value, double denom)
{
    return denom > 1e-9 ? value / denom : 0.0;
}

void addTiming(std::vector<TimingRow>& rows, std::string name, double timeMs, std::string detail = {})
{
    rows.push_back(TimingRow{std::move(name), std::max(0.0, timeMs), std::move(detail)});
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

std::string formatMs(double value)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3) << value;
    return ss.str();
}

} // namespace

bool CoarseMatchProfiler::writeAll(const CoarseMatchReport& report,
                                   const CoarseMatchConfig& config,
                                   const std::filesystem::path& reportsDir) const
{
    if (!config.enableProfiler) {
        return true;
    }
    std::filesystem::create_directories(reportsDir);

    nlohmann::json root;
    root["config"] = config.toString();
    root["currentPreset"] = CoarseMatchConfig::presetName(config.currentPreset);
    root["totalTimeMs"] = report.profile.totalTimeMs;
    root["pyramidBuildTimeMs"] = report.profile.pyramidBuildTimeMs;
    root["distanceFieldBuildTimeMs"] = report.profile.distanceFieldBuildTimeMs;
    root["templatePyramidBuildTimeMs"] = report.profile.templatePyramidBuildTimeMs;
    root["globalSearchTimeMs"] = report.profile.globalSearchTimeMs;
    root["votingTimeMs"] = report.profile.votingTimeMs;
    root["buildRtTableMs"] = report.profile.buildRtTableMs;
    root["sampleImageEdgesMs"] = report.profile.sampleImageEdgesMs;
    root["votingMs"] = report.profile.votingAccumulatorMs;
    root["peakFindMs"] = report.profile.peakFindMs;
    root["votingVerifyMs"] = report.profile.votingVerifyMs;
    root["localRefineTimeMs"] = report.profile.localRefineTimeMs;
    root["fastScoreTimeMs"] = report.profile.fastScoreTimeMs;
    root["nmsTimeMs"] = report.profile.nmsTimeMs;
    root["finalRankerTimeMs"] = report.profile.finalRankerTimeMs;
    root["finalRankerProfile"] = {
        {"totalFinalRankerTimeMs", report.profile.finalRankerProfile.totalFinalRankerTimeMs},
        {"candidateCountBeforeNms", report.profile.finalRankerProfile.candidateCountBeforeNms},
        {"candidateCountAfterNms", report.profile.finalRankerProfile.candidateCountAfterNms},
        {"candidateCountActuallyEvaluated", report.profile.finalRankerProfile.candidateCountActuallyEvaluated},
        {"evaluationMode", report.profile.finalRankerProfile.evaluationMode},
        {"pointEvalEnabled", report.profile.finalRankerProfile.pointEvalEnabled},
        {"pointEvalTopK", report.profile.finalRankerProfile.pointEvalTopK},
        {"distanceFieldValid", report.profile.finalRankerProfile.distanceFieldValid},
        {"roiDistanceFieldQueryCount", report.profile.finalRankerProfile.roiDistanceFieldQueryCount},
        {"fullDistanceFieldQueryCount", report.profile.finalRankerProfile.fullDistanceFieldQueryCount},
        {"linearScanFallbackCount", report.profile.finalRankerProfile.linearScanFallbackCount},
        {"localWindowFallbackCount", report.profile.finalRankerProfile.localWindowFallbackCount},
        {"missingFieldCount", report.profile.finalRankerProfile.missingFieldCount},
        {"numThreads", report.profile.finalRankerProfile.numThreads},
        {"scoreOnlyTimeMs", report.profile.finalRankerProfile.scoreOnlyTimeMs},
        {"pointEvalTimeMs", report.profile.finalRankerProfile.pointEvalTimeMs},
        {"avgTimePerCandidateMs", report.profile.finalRankerProfile.avgTimePerCandidateMs},
        {"avgTimePerTemplatePointUs", report.profile.finalRankerProfile.avgTimePerTemplatePointUs}
    };
    root["reportWriteTimeMs"] = report.profile.reportWriteTimeMs;
    root["overlayBuildTimeMs"] = report.profile.overlayBuildTimeMs;
    root["upperBoundPrunedCount"] = report.profile.upperBoundPrunedCount;
    root["totalEvaluatedPoints"] = report.profile.totalEvaluatedPoints;
    root["totalEvaluatedCandidates"] = report.profile.totalEvaluatedCandidates;
    root["avgEvaluatedPointsPerCandidate"] = report.profile.avgEvaluatedPointsPerCandidate;
    root["candidateReductionRatio"] = report.profile.candidateReductionRatio;
    root["level0CandidateCountBefore"] = report.profile.level0CandidateCountBefore;
    root["level0CandidateCountAfterBudget"] = report.profile.level0CandidateCountAfterBudget;
    root["level0ScoredCandidateCount"] = report.profile.level0ScoredCandidateCount;
    root["soaBuildTimeMs"] = report.profile.soaBuildTimeMs;
    root["rotatedCacheBuildTimeMs"] = report.profile.rotatedCacheBuildTimeMs;
    root["parallelScoringTimeMs"] = report.profile.parallelScoringTimeMs;
    root["mergeTimeMs"] = report.profile.mergeTimeMs;
    root["largeImageMode"] = report.profile.largeImageMode;
    root["skippedFullLevel0DistanceField"] = report.profile.skippedFullLevel0DistanceField;
    root["roiDistanceFieldBuildTimeMs"] = report.profile.roiDistanceFieldBuildTimeMs;
    root["roiDistanceFieldCount"] = report.profile.roiDistanceFieldCount;
    root["roiDistanceFieldTotalPixels"] = report.profile.roiDistanceFieldTotalPixels;

    for (const CoarseMatchLevelResult& level : report.levels) {
        nlohmann::json jLevel = {
            {"level", level.level},
            {"imageWidth", level.imageWidth},
            {"imageHeight", level.imageHeight},
            {"templatePointCount", level.templatePoints},
            {"angleStepDeg", level.angleStepDeg},
            {"translationStepPx", level.translationStepPx},
            {"inputBeamCount", level.inputBeamCount},
            {"generatedCandidateCount", level.generatedCandidateCount},
            {"evaluatedCandidateCount", level.evaluatedCandidateCount},
            {"earlyExitCandidateCount", level.earlyExitCandidateCount},
            {"nmsBeforeCount", level.nmsBeforeCount},
            {"nmsAfterCount", level.nmsAfterCount},
            {"keptCandidateCount", level.keptCandidateCount},
            {"bestFastScore", level.bestFastScore},
            {"worstKeptFastScore", level.worstKeptFastScore},
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
        for (const CoarseMatchAngleStat& angle : level.angleStats) {
            jLevel["angles"].push_back({
                {"angleDeg", angle.angleDeg},
                {"evaluatedCandidateCount", angle.evaluatedCandidateCount},
                {"keptCandidateCount", angle.keptCandidateCount},
                {"bestFastScore", angle.bestFastScore},
                {"elapsedMs", angle.elapsedMs}
            });
        }
        for (const CoarseMatchCellStat& cell : level.cellStats) {
            jLevel["gridCells"].push_back({
                {"row", cell.row},
                {"col", cell.col},
                {"candidateCount", cell.candidateCount},
                {"keptCount", cell.keptCount}
            });
        }
        root["levels"].push_back(std::move(jLevel));
    }

    {
        std::ofstream file(reportsDir / "latest_coarse_profile.json");
        if (!file) {
            return false;
        }
        file << root.dump(2);
    }

    {
        std::ofstream file(reportsDir / "latest_coarse_profile.csv");
        if (!file) {
            return false;
        }
        file << "level,image_width,image_height,template_points,angle_step_deg,translation_step_px,input_beam,"
                "generated,evaluated,raw_generated,after_prefilter,after_budget,scored,parent_before,parent_after,"
                "budget_clipped,max_children_per_parent,avg_children_per_parent,candidate_reduction_ratio,"
                "early_exit,nms_before,nms_after,kept,best_fast,worst_kept,elapsed_ms,"
                "fast_score_ms,nms_ms,scoring_mode,scorer_mode,distance_field_valid,distance_field_build_ms,"
                "local_fallback,upper_bound_pruned,avg_eval_points,full_eval,pruned,"
                "soa_build_ms,rotated_cache_build_ms,rotated_cache_hit_rate,parallel_scoring_ms,num_scoring_threads,merge_ms,"
                "candidate_source,"
                "voting_raw,voting_scored,voting_time_ms,voting_verify_ms,grid_fallback\n";
        file << std::fixed << std::setprecision(6);
        for (const CoarseMatchLevelResult& level : report.levels) {
            file << level.level << ',' << level.imageWidth << ',' << level.imageHeight << ','
                 << level.templatePoints << ',' << level.angleStepDeg << ',' << level.translationStepPx << ','
                 << level.inputBeamCount << ',' << level.generatedCandidateCount << ',' << level.evaluatedCandidateCount << ','
                 << level.rawGeneratedCandidates << ',' << level.afterPreFilterCandidates << ',' << level.afterBudgetCandidates << ','
                 << level.scoredCandidates << ',' << level.parentCountBeforeSelection << ',' << level.parentCountAfterSelection << ','
                 << level.budgetClippedCount << ',' << level.maxChildrenPerParent << ',' << level.avgChildrenPerParent << ','
                 << level.candidateReductionRatio << ','
                 << level.earlyExitCandidateCount << ',' << level.nmsBeforeCount << ',' << level.nmsAfterCount << ','
                 << level.keptCandidateCount << ',' << level.bestFastScore << ',' << level.worstKeptFastScore << ','
                 << level.elapsedMs << ',' << level.fastScoreTimeMs << ',' << level.nmsTimeMs << ','
                 << level.scoringMode << ',' << level.scorerMode << ',' << (level.distanceFieldValid ? "true" : "false") << ','
                 << level.distanceFieldBuildTimeMs << ',' << level.localSearchFallbackCount << ','
                 << level.upperBoundPrunedCount << ',' << level.avgEvaluatedPointsPerCandidate << ','
                 << level.fullEvaluatedCandidateCount << ',' << level.prunedCandidateCount << ','
                 << level.soaBuildTimeMs << ',' << level.rotatedCacheBuildTimeMs << ',' << level.rotatedCacheHitRate << ','
                 << level.parallelScoringTimeMs << ',' << level.numScoringThreads << ',' << level.mergeTimeMs << ','
                 << level.candidateSource << ','
                 << level.votingRawCandidateCount << ',' << level.votingScoredCandidateCount << ','
                 << level.votingTimeMs << ',' << level.votingVerifyTimeMs << ','
                 << (level.gridFallbackUsed ? "true" : "false") << '\n';
        }
    }

    {
        const FinalRankerProfile& fp = report.profile.finalRankerProfile;
        nlohmann::json finalRoot = {
            {"totalFinalRankerTimeMs", fp.totalFinalRankerTimeMs},
            {"candidateCountBeforeNms", fp.candidateCountBeforeNms},
            {"candidateCountAfterNms", fp.candidateCountAfterNms},
            {"candidateCountActuallyEvaluated", fp.candidateCountActuallyEvaluated},
            {"evaluationMode", fp.evaluationMode},
            {"pointEvalEnabled", fp.pointEvalEnabled},
            {"pointEvalTopK", fp.pointEvalTopK},
        {"distanceFieldValid", fp.distanceFieldValid},
        {"roiDistanceFieldQueryCount", fp.roiDistanceFieldQueryCount},
        {"fullDistanceFieldQueryCount", fp.fullDistanceFieldQueryCount},
        {"linearScanFallbackCount", fp.linearScanFallbackCount},
        {"localWindowFallbackCount", fp.localWindowFallbackCount},
        {"missingFieldCount", fp.missingFieldCount},
            {"numThreads", fp.numThreads},
            {"scoreOnlyTimeMs", fp.scoreOnlyTimeMs},
            {"pointEvalTimeMs", fp.pointEvalTimeMs},
            {"avgTimePerCandidateMs", fp.avgTimePerCandidateMs},
            {"avgTimePerTemplatePointUs", fp.avgTimePerTemplatePointUs}
        };
        std::ofstream file(reportsDir / "latest_final_ranker_profile.json");
        if (!file) {
            return false;
        }
        file << finalRoot.dump(2);
    }

    {
        const FinalRankerProfile& fp = report.profile.finalRankerProfile;
        std::ofstream file(reportsDir / "latest_final_ranker_profile.csv");
        if (!file) {
            return false;
        }
        file << "totalFinalRankerTimeMs,candidateCountBeforeNms,candidateCountAfterNms,"
                "candidateCountActuallyEvaluated,evaluationMode,pointEvalEnabled,pointEvalTopK,"
                "distanceFieldValid,roiDistanceFieldQueryCount,fullDistanceFieldQueryCount,linearScanFallbackCount,localWindowFallbackCount,missingFieldCount,numThreads,"
                "scoreOnlyTimeMs,pointEvalTimeMs,avgTimePerCandidateMs,avgTimePerTemplatePointUs\n";
        file << std::fixed << std::setprecision(6)
             << fp.totalFinalRankerTimeMs << ','
             << fp.candidateCountBeforeNms << ','
             << fp.candidateCountAfterNms << ','
             << fp.candidateCountActuallyEvaluated << ','
             << csvEscape(fp.evaluationMode) << ','
             << (fp.pointEvalEnabled ? "true" : "false") << ','
             << fp.pointEvalTopK << ','
             << (fp.distanceFieldValid ? "true" : "false") << ','
             << fp.roiDistanceFieldQueryCount << ','
             << fp.fullDistanceFieldQueryCount << ','
             << fp.linearScanFallbackCount << ','
             << fp.localWindowFallbackCount << ','
             << fp.missingFieldCount << ','
             << fp.numThreads << ','
             << fp.scoreOnlyTimeMs << ','
             << fp.pointEvalTimeMs << ','
             << fp.avgTimePerCandidateMs << ','
             << fp.avgTimePerTemplatePointUs << '\n';
    }

    {
        nlohmann::json roiRoot;
        roiRoot["largeImageMode"] = report.profile.largeImageMode;
        roiRoot["skippedFullLevel0DistanceField"] = report.profile.skippedFullLevel0DistanceField;
        roiRoot["roiDistanceFieldCount"] = report.profile.roiDistanceFieldCount;
        roiRoot["roiDistanceFieldTotalPixels"] = report.profile.roiDistanceFieldTotalPixels;
        roiRoot["roiDistanceFieldBuildTimeMs"] = report.profile.roiDistanceFieldBuildTimeMs;
        for (const RoiDistanceFieldProfileRow& row : report.roiDistanceFieldRows) {
            roiRoot["rois"].push_back({
                {"level", row.level},
                {"roi_id", row.roiId},
                {"roi_x", row.x},
                {"roi_y", row.y},
                {"roi_w", row.width},
                {"roi_h", row.height},
                {"pixel_count", row.pixelCount},
                {"build_time_ms", row.buildTimeMs},
                {"valid", row.valid},
                {"source_candidate_count", row.sourceCandidateCount}
            });
        }
        std::ofstream file(reportsDir / "latest_roi_distance_field_profile.json");
        if (!file) {
            return false;
        }
        file << roiRoot.dump(2);
    }

    {
        std::ofstream file(reportsDir / "latest_roi_distance_field_profile.csv");
        if (!file) {
            return false;
        }
        file << "level,roi_id,roi_x,roi_y,roi_w,roi_h,pixel_count,build_time_ms,valid,source_candidate_count\n";
        file << std::fixed << std::setprecision(6);
        for (const RoiDistanceFieldProfileRow& row : report.roiDistanceFieldRows) {
            file << row.level << ',' << row.roiId << ',' << row.x << ',' << row.y << ','
                 << row.width << ',' << row.height << ',' << row.pixelCount << ','
                 << row.buildTimeMs << ',' << (row.valid ? "true" : "false") << ','
                 << row.sourceCandidateCount << '\n';
        }
    }

    {
        std::ofstream file(reportsDir / "latest_level0_budget_v2.csv");
        if (!file) {
            return false;
        }
        file << "level,parent_before,parent_after,raw_children,after_region_filter,after_duplicate_filter,"
                "after_budget,scored,budget_clipped,region_rejected,duplicate_rejected\n";
        for (const CoarseMatchLevelResult& level : report.levels) {
            if (level.level != 0) {
                continue;
            }
            file << level.level << ',' << level.parentCountBeforeSelection << ',' << level.parentCountAfterSelection << ','
                 << level.rawGeneratedCandidates << ',' << level.level0ChildrenAfterRegionFilter << ','
                 << level.level0ChildrenAfterDuplicateFilter << ',' << level.afterBudgetCandidates << ','
                 << level.evaluatedCandidateCount << ',' << level.budgetClippedCount << ','
                 << level.level0RegionRejectedChildren << ',' << level.level0DuplicateRejectedChildren << '\n';
        }
    }

    {
        std::ofstream file(reportsDir / "latest_large_image_summary.txt");
        if (!file) {
            return false;
        }
        file << std::fixed << std::setprecision(3);
        file << "Large Image Shape Match Summary\n";
        file << "largeImageMode: " << (report.profile.largeImageMode ? "true" : "false") << "\n";
        file << "buildFullDistanceFieldForLevel0: " << (config.roiDistanceField.buildFullDistanceFieldForLevel0 ? "true" : "false") << "\n";
        file << "skippedFullLevel0DistanceField: " << (report.profile.skippedFullLevel0DistanceField ? "true" : "false") << "\n";
        file << "fullDistanceFieldBuildTimeMsTotal: " << report.profile.distanceFieldBuildTimeMs << "\n";
        for (const CoarseMatchLevelResult& level : report.levels) {
            file << "fullDistanceFieldBuildTimeMsLevel" << level.level << ": " << level.distanceFieldBuildTimeMs << "\n";
        }
        file << "roiDistanceFieldCount: " << report.profile.roiDistanceFieldCount << "\n";
        file << "roiDistanceFieldTotalPixels: " << report.profile.roiDistanceFieldTotalPixels << "\n";
        file << "roiDistanceFieldBuildTimeMs: " << report.profile.roiDistanceFieldBuildTimeMs << "\n";
        file << "level0ParentBefore: " << report.profile.level0ParentBefore << "\n";
        file << "level0ParentAfter: " << report.profile.level0ParentAfter << "\n";
        file << "level0RawChildren: " << report.profile.level0RawChildren << "\n";
        file << "level0ScoredChildren: " << report.profile.level0ScoredChildren << "\n";
        file << "localRefineTotalMs: " << report.profile.localRefineTimeMs << "\n";
        file << "fastScoreTotalMs: " << report.profile.fastScoreTimeMs << "\n";
        file << "finalRankerMs: " << report.profile.finalRankerTimeMs << "\n";
        file << "totalTimeMs: " << report.profile.totalTimeMs << "\n";
        file << "recall: " << report.multiTargetEval.recall << "\n";
        file << "top1Hit: " << (report.multiTargetEval.top1Hit ? "true" : "false") << "\n";
        file << "topKHit: " << (report.multiTargetEval.top10Hit ? "true" : "false") << "\n";
        file << "roiDistanceFieldQueryCount: " << report.profile.finalRankerProfile.roiDistanceFieldQueryCount << "\n";
        file << "fullDistanceFieldQueryCount: " << report.profile.finalRankerProfile.fullDistanceFieldQueryCount << "\n";
        file << "localWindowFallbackCount: " << report.profile.finalRankerProfile.localWindowFallbackCount << "\n";
        file << "linearScanFallbackCount: " << report.profile.finalRankerProfile.linearScanFallbackCount << "\n";
        file << "missingFieldCount: " << report.profile.finalRankerProfile.missingFieldCount << "\n";
    }

    std::vector<TimingRow> phaseRows;
    addTiming(phaseRows, "total_match", report.profile.totalTimeMs, "End-to-end matcher time before report writing");
    addTiming(phaseRows, "image_pyramid", report.profile.pyramidBuildTimeMs, "Build image pyramid");
    addTiming(phaseRows, "template_pyramid", report.profile.templatePyramidBuildTimeMs, "Build template pyramid");
    addTiming(phaseRows, "distance_field_build", report.profile.distanceFieldBuildTimeMs, "Build per-level distance/orientation fields");
    addTiming(phaseRows, "orientation_voting_total", report.profile.votingTimeMs, "RT voting candidate generation");
    addTiming(phaseRows, "rt_table_build", report.profile.buildRtTableMs, "Build RT table for voting");
    addTiming(phaseRows, "image_edge_sampling", report.profile.sampleImageEdgesMs, "Sample image edge points for voting");
    addTiming(phaseRows, "voting_accumulator", report.profile.votingAccumulatorMs, "Accumulator voting");
    addTiming(phaseRows, "peak_find", report.profile.peakFindMs, "Accumulator peak extraction");
    addTiming(phaseRows, "voting_verify", report.profile.votingVerifyMs, "Fast score voting peaks");
    addTiming(phaseRows, "local_refine_total", report.profile.localRefineTimeMs, "Parent/child generation and per-level fast scoring");
    addTiming(phaseRows, "fast_score_total", report.profile.fastScoreTimeMs, "FastPoseScorer across all coarse levels");
    addTiming(phaseRows, "parallel_fast_score", report.profile.parallelScoringTimeMs, "Parallel candidate scoring worker time");
    addTiming(phaseRows, "nms_total", report.profile.nmsTimeMs, "TopK/NMS/spatial diversity");
    addTiming(phaseRows, "soa_build", report.profile.soaBuildTimeMs, "Template SoA build");
    addTiming(phaseRows, "rotated_cache_build", report.profile.rotatedCacheBuildTimeMs, "Rotated template cache build");
    addTiming(phaseRows, "parallel_merge", report.profile.mergeTimeMs, "Parallel scoring merge");
    addTiming(phaseRows, "final_ranker", report.profile.finalRankerTimeMs, "MatchEvaluator final candidate ranking");
    addTiming(phaseRows, "report_write", report.profile.reportWriteTimeMs, "Report writer time, not included in total_match");

    std::vector<TimingRow> bottlenecks = phaseRows;
    bottlenecks.erase(std::remove_if(bottlenecks.begin(),
                                     bottlenecks.end(),
                                     [](const TimingRow& row) {
                                         return row.name == "total_match" || row.name == "report_write" || row.timeMs <= 0.0;
                                     }),
                      bottlenecks.end());
    std::sort(bottlenecks.begin(), bottlenecks.end(), [](const TimingRow& a, const TimingRow& b) {
        return a.timeMs > b.timeMs;
    });

    nlohmann::json timingRoot;
    timingRoot["config"] = config.toString();
    timingRoot["currentPreset"] = CoarseMatchConfig::presetName(config.currentPreset);
    timingRoot["totalTimeMs"] = report.profile.totalTimeMs;
    timingRoot["finalCandidateCount"] = static_cast<int>(report.finalRankedCandidates.size());
    timingRoot["coarseFinalCandidateCount"] = static_cast<int>(report.finalCoarseCandidates.size());
    timingRoot["totalEvaluatedCandidates"] = report.profile.totalEvaluatedCandidates;
    timingRoot["totalEvaluatedPoints"] = report.profile.totalEvaluatedPoints;
    timingRoot["avgEvaluatedPointsPerCandidate"] = report.profile.avgEvaluatedPointsPerCandidate;
    timingRoot["level0CandidateCountBefore"] = report.profile.level0CandidateCountBefore;
    timingRoot["level0CandidateCountAfterBudget"] = report.profile.level0CandidateCountAfterBudget;
    timingRoot["candidateReductionRatio"] = report.profile.candidateReductionRatio;
    timingRoot["finalRankerProfile"] = {
        {"evaluationMode", report.profile.finalRankerProfile.evaluationMode},
        {"candidateCountBeforeNms", report.profile.finalRankerProfile.candidateCountBeforeNms},
        {"candidateCountAfterNms", report.profile.finalRankerProfile.candidateCountAfterNms},
        {"candidateCountActuallyEvaluated", report.profile.finalRankerProfile.candidateCountActuallyEvaluated},
        {"pointEvalEnabled", report.profile.finalRankerProfile.pointEvalEnabled},
        {"pointEvalTopK", report.profile.finalRankerProfile.pointEvalTopK},
        {"linearScanFallbackCount", report.profile.finalRankerProfile.linearScanFallbackCount},
        {"localWindowFallbackCount", report.profile.finalRankerProfile.localWindowFallbackCount},
        {"numThreads", report.profile.finalRankerProfile.numThreads},
        {"scoreOnlyTimeMs", report.profile.finalRankerProfile.scoreOnlyTimeMs},
        {"pointEvalTimeMs", report.profile.finalRankerProfile.pointEvalTimeMs},
        {"avgTimePerCandidateMs", report.profile.finalRankerProfile.avgTimePerCandidateMs},
        {"avgTimePerTemplatePointUs", report.profile.finalRankerProfile.avgTimePerTemplatePointUs}
    };
    timingRoot["msPerFinalRankedCandidate"] = safeDiv(report.profile.finalRankerTimeMs,
                                                       static_cast<double>(std::max<size_t>(1, report.finalRankedCandidates.size())));
    timingRoot["msPerFastScoredCandidate"] = safeDiv(report.profile.fastScoreTimeMs,
                                                      static_cast<double>(std::max(1, report.profile.totalEvaluatedCandidates)));
    timingRoot["bottleneck"] = bottlenecks.empty() ? "" : bottlenecks.front().name;
    timingRoot["bottleneckPercent"] = bottlenecks.empty() ? 0.0 : percentOf(bottlenecks.front().timeMs, report.profile.totalTimeMs);
    for (const TimingRow& row : phaseRows) {
        timingRoot["phases"].push_back({
            {"name", row.name},
            {"timeMs", row.timeMs},
            {"percentOfTotal", percentOf(row.timeMs, report.profile.totalTimeMs)},
            {"detail", row.detail}
        });
    }
    for (size_t i = 0; i < std::min<size_t>(5, bottlenecks.size()); ++i) {
        timingRoot["topBottlenecks"].push_back({
            {"rank", static_cast<int>(i + 1)},
            {"name", bottlenecks[i].name},
            {"timeMs", bottlenecks[i].timeMs},
            {"percentOfTotal", percentOf(bottlenecks[i].timeMs, report.profile.totalTimeMs)},
            {"detail", bottlenecks[i].detail}
        });
    }
    for (const CoarseMatchLevelResult& level : report.levels) {
        timingRoot["levels"].push_back({
            {"level", level.level},
            {"elapsedMs", level.elapsedMs},
            {"percentOfTotal", percentOf(level.elapsedMs, report.profile.totalTimeMs)},
            {"evaluatedCandidates", level.evaluatedCandidateCount},
            {"rawGeneratedCandidates", level.rawGeneratedCandidates},
            {"afterBudgetCandidates", level.afterBudgetCandidates},
            {"keptCandidates", level.keptCandidateCount},
            {"fastScoreMs", level.fastScoreTimeMs},
            {"nmsMs", level.nmsTimeMs},
            {"parallelScoringMs", level.parallelScoringTimeMs},
            {"distanceFieldBuildMs", level.distanceFieldBuildTimeMs},
            {"candidateLimitHit", level.candidateLimitHit},
            {"budgetClippedCount", level.budgetClippedCount},
            {"msPerEvaluatedCandidate", safeDiv(level.fastScoreTimeMs, static_cast<double>(std::max(1, level.evaluatedCandidateCount)))},
            {"scorerMode", level.scorerMode},
            {"candidateSource", level.candidateSource}
        });
    }

    {
        std::ofstream file(reportsDir / "latest_algorithm_timing_report.json");
        if (!file) {
            return false;
        }
        file << timingRoot.dump(2);
    }

    {
        std::ofstream file(reportsDir / "latest_algorithm_timing_report.csv");
        if (!file) {
            return false;
        }
        file << "section,name,time_ms,percent_of_total,detail\n";
        file << std::fixed << std::setprecision(6);
        for (const TimingRow& row : phaseRows) {
            file << "phase," << csvEscape(row.name) << ',' << row.timeMs << ','
                 << percentOf(row.timeMs, report.profile.totalTimeMs) << ','
                 << csvEscape(row.detail) << '\n';
        }
        for (size_t i = 0; i < std::min<size_t>(5, bottlenecks.size()); ++i) {
            file << "bottleneck_" << (i + 1) << ',' << csvEscape(bottlenecks[i].name) << ','
                 << bottlenecks[i].timeMs << ','
                 << percentOf(bottlenecks[i].timeMs, report.profile.totalTimeMs) << ','
                 << csvEscape(bottlenecks[i].detail) << '\n';
        }
    }

    {
        std::ofstream file(reportsDir / "latest_algorithm_timing_report.txt");
        if (!file) {
            return false;
        }
        file << std::fixed << std::setprecision(3);
        file << "Algorithm Timing Report\n";
        file << "Total match time: " << report.profile.totalTimeMs << " ms\n";
        file << "Final ranked candidates: " << report.finalRankedCandidates.size() << "\n";
        file << "Total evaluated coarse candidates: " << report.profile.totalEvaluatedCandidates << "\n";
        file << "Level0 candidates before/after budget: "
             << report.profile.level0CandidateCountBefore << '/'
             << report.profile.level0CandidateCountAfterBudget << "\n";
        file << "Fast scoring cost: "
             << safeDiv(report.profile.fastScoreTimeMs, static_cast<double>(std::max(1, report.profile.totalEvaluatedCandidates)))
             << " ms/evaluated candidate\n";
        file << "Final ranking cost: "
             << safeDiv(report.profile.finalRankerTimeMs, static_cast<double>(std::max<size_t>(1, report.finalRankedCandidates.size())))
             << " ms/final candidate\n\n";
        file << "Final ranker profile: mode=" << report.profile.finalRankerProfile.evaluationMode
             << ", candidates=" << report.profile.finalRankerProfile.candidateCountBeforeNms
             << "->" << report.profile.finalRankerProfile.candidateCountAfterNms
             << ", threads=" << report.profile.finalRankerProfile.numThreads
             << ", linearFallback=" << report.profile.finalRankerProfile.linearScanFallbackCount
             << ", localFallback=" << report.profile.finalRankerProfile.localWindowFallbackCount
             << ", pointEvalMs=" << report.profile.finalRankerProfile.pointEvalTimeMs
             << ", avgPointUs=" << report.profile.finalRankerProfile.avgTimePerTemplatePointUs
             << "\n\n";

        file << "Top bottlenecks:\n";
        for (size_t i = 0; i < std::min<size_t>(5, bottlenecks.size()); ++i) {
            file << (i + 1) << ". " << bottlenecks[i].name
                 << ": " << bottlenecks[i].timeMs << " ms ("
                 << percentOf(bottlenecks[i].timeMs, report.profile.totalTimeMs) << "%) - "
                 << bottlenecks[i].detail << "\n";
        }

        file << "\nPer-level timing:\n";
        for (const CoarseMatchLevelResult& level : report.levels) {
            file << "- level " << level.level
                 << ": elapsed=" << level.elapsedMs << " ms"
                 << ", evaluated=" << level.evaluatedCandidateCount
                 << ", kept=" << level.keptCandidateCount
                 << ", fastScore=" << level.fastScoreTimeMs << " ms"
                 << ", nms=" << level.nmsTimeMs << " ms"
                 << ", ms/eval=" << safeDiv(level.fastScoreTimeMs, static_cast<double>(std::max(1, level.evaluatedCandidateCount)))
                 << ", parents=" << level.parentCountBeforeSelection << "->" << level.parentCountAfterSelection
                 << ", budgetClipped=" << level.budgetClippedCount
                 << ", mode=" << level.scorerMode << "\n";
        }

        file << "\nDiagnosis:\n";
        if (!bottlenecks.empty() && bottlenecks.front().name == "final_ranker") {
            file << "- Primary bottleneck is final ranking / MatchEvaluator.\n";
            file << "- Final ranker now reports evaluator mode, candidate NMS, point eval cost, and fallback counts. If linearFallback is non-zero on large images, inspect distance field validity.\n";
        } else if (!bottlenecks.empty() && bottlenecks.front().name == "distance_field_build") {
            file << "- Primary bottleneck is distance field construction. Consider caching pyramid fields for unchanged images or building fewer levels.\n";
        } else if (!bottlenecks.empty() && bottlenecks.front().name == "local_refine_total") {
            file << "- Primary bottleneck is local refine. Reduce parent beam, child window, or candidate budgets after recall is stable.\n";
        } else {
            file << "- No single dominant bottleneck detected; inspect top phases and per-level rows.\n";
        }
    }
    return true;
}

} // namespace ShapeMatch
