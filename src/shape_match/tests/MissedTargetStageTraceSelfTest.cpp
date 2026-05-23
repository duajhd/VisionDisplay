#include "shape_match/coarse/MissedTargetStageTrace.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {

using namespace ShapeMatch;

CandidateTraceInfo stageInfo(const std::string& stage, bool nearTarget, const std::string& reason = {})
{
    CandidateTraceInfo info;
    info.candidateId = 1;
    info.source = stage;
    info.level = 0;
    info.rank = 0;
    info.pose = MatchPose::fromDeg(100.0 + (nearTarget ? 1.0 : 50.0), 120.0, nearTarget ? 2.0 : 35.0);
    info.exists = true;
    info.kept = reason.empty();
    info.dxy = nearTarget ? 1.0 : 50.0;
    info.dthetaDeg = nearTarget ? 2.0 : 35.0;
    info.fastScore = nearTarget ? 0.82 : 0.10;
    info.rejectReason = reason;
    info.clippedByBudget = reason == "budget_clipped";
    info.suppressedByNms = reason == "nms_suppressed";
    info.prunedByUpperBound = reason == "upper_bound_pruned";
    return info;
}

MissedTargetStageInfo target(const std::string& id,
                             bool finalHit,
                             const std::string& firstMissingStage,
                             const std::string& likelyReason)
{
    MissedTargetStageInfo info;
    info.targetId = id;
    info.expectedPose = MatchPose::fromDeg(100.0, 120.0, 0.0);
    info.hitFinalTopK = finalHit;
    info.hitFinalRanked = finalHit;
    info.firstMissingStage = firstMissingStage;
    info.likelyReason = likelyReason;
    info.oracleFastScoreLevel0 = likelyReason == "fast_score_model_unfriendly" ? 0.04 : 0.84;
    info.oracleFastScoreTopLevel = 0.76;
    info.rawEdgeCountInRoi = likelyReason == "image_edge_sampler_dropped_roi_points" ? 420 : 120;
    info.sampledVotePointCountInRoi = likelyReason == "image_edge_sampler_dropped_roi_points" ? 3 : 36;
    info.roiVoteStats.rawEdgeCountInRoi = info.rawEdgeCountInRoi;
    info.roiVoteStats.sampledVotePointCountInRoi = info.sampledVotePointCountInRoi;
    info.roiVoteStats.votePointKeepRatioInRoi = info.rawEdgeCountInRoi > 0
        ? static_cast<double>(info.sampledVotePointCountInRoi) / static_cast<double>(info.rawEdgeCountInRoi)
        : 0.0;
    info.nearestByStage.push_back(stageInfo("voting_verified", true));
    info.nearestByStage.push_back(stageInfo(firstMissingStage.empty() ? "final_ranked_candidates" : firstMissingStage,
                                            finalHit,
                                            likelyReason));
    info.diagnosis = finalHit
        ? "Target reached final topK."
        : "Target missed at " + firstMissingStage + " likelyReason=" + likelyReason + ".";
    return info;
}

std::string readFile(const std::filesystem::path& path)
{
    std::ifstream file(path);
    std::ostringstream out;
    out << file.rdbuf();
    return out.str();
}

bool contains(const std::string& haystack, const std::string& needle)
{
    return haystack.find(needle) != std::string::npos;
}

} // namespace

int main()
{
    const std::filesystem::path reportsDir = std::filesystem::path("data") / "shape_match" / "reports";
    MissedTargetStageTraceReport report;
    report.imageName = "selftest";
    report.templateName = "synthetic";
    report.nearDxyThresholdPx = 15.0;
    report.nearAngleThresholdDeg = 15.0;
    report.targets.push_back(target("budget_clip", false, "after_candidate_budget", "budget_clipped"));
    report.targets.push_back(target("nms_suppress", false, "after_nms", "nms_suppressed"));
    report.targets.push_back(target("upper_bound", false, "after_upper_bound_pruning", "upper_bound_pruned"));
    report.targets.push_back(target("sampler_drop", false, "voting_raw_peaks", "image_edge_sampler_dropped_roi_points"));
    report.targets.push_back(target("normal_hit", true, "", ""));

    MissedTargetStageTraceWriter writer;
    if (!writer.writeAll(report, reportsDir)) {
        std::cerr << "failed to write stage trace report\n";
        return 1;
    }

    const std::filesystem::path jsonPath = reportsDir / "latest_missed_target_stage_trace.json";
    const std::filesystem::path csvPath = reportsDir / "latest_missed_target_stage_trace.csv";
    const std::filesystem::path txtPath = reportsDir / "latest_missed_target_stage_trace.txt";
    if (!std::filesystem::exists(jsonPath) || !std::filesystem::exists(csvPath) || !std::filesystem::exists(txtPath)) {
        std::cerr << "missing stage trace output files\n";
        return 2;
    }

    const std::string json = readFile(jsonPath);
    const std::string csv = readFile(csvPath);
    const std::string txt = readFile(txtPath);
    const char* required[] = {
        "budget_clipped",
        "nms_suppressed",
        "upper_bound_pruned",
        "image_edge_sampler_dropped_roi_points",
        "normal_hit"
    };
    for (const char* token : required) {
        if (!contains(json, token) || !contains(csv, token) || !contains(txt, token)) {
            std::cerr << "missing token in reports: " << token << "\n";
            return 3;
        }
    }

    std::cout << "MissedTargetStageTraceSelfTest passed\n";
    return 0;
}
