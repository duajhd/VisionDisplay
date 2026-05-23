#include "shape_match/coarse/MissedTargetDiagnostic.h"

#include "shape_match/coarse/CoarsePoseGenerator.h"
#include "shape_match/coarse/FastPoseScorer.h"
#include "shape_match/evaluation/PoseErrorEvaluator.h"

#include <nlohmann/json.hpp>

#include <fstream>
#include <iomanip>

namespace ShapeMatch {

namespace {

struct NearestInfo
{
    bool found = false;
    double dxy = 0.0;
    double dthetaDeg = 0.0;
    double fastScore = 0.0;
};

NearestInfo nearestCoarse(const std::vector<CoarseCandidate>& candidates, const MatchPose& pose)
{
    NearestInfo out;
    double bestCost = std::numeric_limits<double>::max();
    for (const CoarseCandidate& c : candidates) {
        const double dx = c.pose.x - pose.x;
        const double dy = c.pose.y - pose.y;
        const double dxy = std::sqrt(dx * dx + dy * dy);
        const double dtheta = std::abs(radToDeg(wrapToPi(c.pose.theta - pose.theta)));
        const double cost = dxy + 0.5 * dtheta;
        if (cost < bestCost) {
            bestCost = cost;
            out.found = true;
            out.dxy = dxy;
            out.dthetaDeg = dtheta;
            out.fastScore = c.fastScore;
        }
    }
    return out;
}

bool finalHit(const std::vector<ScoredCandidate>& candidates, const MatchPose& gt)
{
    ShapeMatchEvalConfig config;
    PoseErrorEvaluator evaluator(config);
    for (const ScoredCandidate& c : candidates) {
        const PoseError e = evaluator.evaluatePoseError(c.pose, gt);
        if (e.poseOk) {
            return true;
        }
    }
    return false;
}

std::string classify(const NearestInfo& finalNearest, double oracleFastScore)
{
    if (!finalNearest.found) {
        return "candidate_not_generated";
    }
    if (finalNearest.dxy > 12.0) {
        return oracleFastScore > 0.20 ? "beam_pruned" : "score_too_low";
    }
    if (finalNearest.dthetaDeg > 8.0) {
        return "angle_grid_miss";
    }
    if (oracleFastScore < 0.10) {
        return "score_too_low";
    }
    return "unknown";
}

} // namespace

bool MissedTargetDiagnostic::writeAll(const CoarseMatchReport& report,
                                      const CoarseMatchConfig& config,
                                      const ImagePyramid& imagePyramid,
                                      const TemplatePyramid& templatePyramid,
                                      const std::filesystem::path& reportsDir) const
{
    if (report.groundTruthInstances.empty()) {
        return true;
    }
    std::filesystem::create_directories(reportsDir);
    nlohmann::json root;
    FastPoseScorer scorer(config);

    std::ofstream txt(reportsDir / "latest_missed_target_diagnostic.txt");
    if (!txt) {
        return false;
    }
    std::ofstream stageTxt(reportsDir / "latest_missed_target_stage_trace.txt");
    if (!stageTxt) {
        return false;
    }
    txt << std::fixed << std::setprecision(4);
    stageTxt << std::fixed << std::setprecision(4);

    for (const GroundTruthInstance& gt : report.groundTruthInstances) {
        const bool hit = finalHit(report.finalRankedCandidates, gt.pose);
        NearestInfo finalNearest;
        std::vector<CoarseCandidate> finalAsCoarse = report.finalCoarseCandidates;
        finalNearest = nearestCoarse(finalAsCoarse, gt.pose);

        nlohmann::json jGt;
        nlohmann::json jStageGt;
        jGt["id"] = gt.id;
        jGt["enteredFinalTopK"] = hit;
        jGt["nearestFinalDxy"] = finalNearest.dxy;
        jGt["nearestFinalDthetaDeg"] = finalNearest.dthetaDeg;
        jStageGt["id"] = gt.id;
        jStageGt["enteredFinalTopK"] = hit;
        jStageGt["nearestFinalDxy"] = finalNearest.dxy;
        jStageGt["nearestFinalDthetaDeg"] = finalNearest.dthetaDeg;
        jStageGt["finalCandidateRankerKept"] = hit;
        txt << "GT " << gt.id << " enteredFinalTopK=" << (hit ? "true" : "false")
            << " nearestFinalDxy=" << finalNearest.dxy
            << " nearestFinalDthetaDeg=" << finalNearest.dthetaDeg << "\n";
        stageTxt << "GT " << gt.id << " enteredFinalTopK=" << (hit ? "true" : "false")
                 << " nearestFinalDxy=" << finalNearest.dxy
                 << " nearestFinalDthetaDeg=" << finalNearest.dthetaDeg << "\n";

        double lastOracle = 0.0;
        for (const CoarseMatchLevelResult& levelResult : report.levels) {
            const int level = levelResult.level;
            const MatchPose gtLevel = CoarsePoseGenerator::convertPoseBetweenLevels(gt.pose, 1.0, imagePyramid.scaleOfLevel(level));
            const NearestInfo nearest = nearestCoarse(levelResult.topCandidates, gtLevel);
            const CoarseCandidate oracle = scorer.scorePose(templatePyramid.level(level),
                                                            imagePyramid.level(level),
                                                            gtLevel,
                                                            level,
                                                            -1.0);
            lastOracle = oracle.fastScore;
            jGt["levels"].push_back({
                {"level", level},
                {"nearestFound", nearest.found},
                {"nearestDxy", nearest.dxy},
                {"nearestDthetaDeg", nearest.dthetaDeg},
                {"nearestFastScore", nearest.fastScore},
                {"oracleFastScore", oracle.fastScore}
            });
            const bool nearby = nearest.found && nearest.dxy <= 12.0 && nearest.dthetaDeg <= 10.0;
            jStageGt["levels"].push_back({
                {"level", level},
                {"candidateSource", levelResult.candidateSource},
                {"votingNearbyPeak", levelResult.candidateSource == "voting" && nearby},
                {"parentSelectionBeforeCount", levelResult.parentCountBeforeSelection},
                {"parentSelectionAfterCount", levelResult.parentCountAfterSelection},
                {"localRefineBeforeHadNearby", nearby},
                {"localRefineAfterHadNearby", nearby},
                {"budgetClippedCount", levelResult.budgetClippedCount},
                {"budgetMayHaveClipped", levelResult.budgetClippedCount > 0 && !nearby},
                {"nmsBeforeCount", levelResult.nmsBeforeCount},
                {"nmsAfterCount", levelResult.nmsAfterCount},
                {"upperBoundPrunedCount", levelResult.upperBoundPrunedCount},
                {"nearestFound", nearest.found},
                {"nearestDxy", nearest.dxy},
                {"nearestDthetaDeg", nearest.dthetaDeg},
                {"nearestFastScore", nearest.fastScore},
                {"oracleFastScore", oracle.fastScore}
            });
            txt << "  level " << level
                << " nearestFound=" << (nearest.found ? "true" : "false")
                << " nearestDxy=" << nearest.dxy
                << " nearestDthetaDeg=" << nearest.dthetaDeg
                << " nearestFastScore=" << nearest.fastScore
                << " oracleFastScore=" << oracle.fastScore << "\n";
            stageTxt << "  level " << level
                     << " source=" << levelResult.candidateSource
                     << " nearby=" << (nearby ? "true" : "false")
                     << " parentSelection=" << levelResult.parentCountBeforeSelection << "->" << levelResult.parentCountAfterSelection
                     << " budgetClipped=" << levelResult.budgetClippedCount
                     << " nms=" << levelResult.nmsBeforeCount << "->" << levelResult.nmsAfterCount
                     << " upperBoundPruned=" << levelResult.upperBoundPrunedCount
                     << " nearestDxy=" << nearest.dxy
                     << " nearestDthetaDeg=" << nearest.dthetaDeg
                     << " nearestFastScore=" << nearest.fastScore
                     << " oracleFastScore=" << oracle.fastScore << "\n";
        }
        jGt["classification"] = classify(finalNearest, lastOracle);
        jStageGt["classification"] = classify(finalNearest, lastOracle);
        txt << "  classification=" << jGt["classification"].get<std::string>() << "\n";
        stageTxt << "  classification=" << jStageGt["classification"].get<std::string>() << "\n";
        root["groundTruth"].push_back(std::move(jGt));
        root["stageTrace"].push_back(std::move(jStageGt));
    }

    std::ofstream jsonFile(reportsDir / "latest_missed_target_diagnostic.json");
    if (!jsonFile) {
        return false;
    }
    jsonFile << root.dump(2);
    std::ofstream stageJsonFile(reportsDir / "latest_missed_target_stage_trace.json");
    if (!stageJsonFile) {
        return false;
    }
    stageJsonFile << root.dump(2);
    return true;
}

} // namespace ShapeMatch
