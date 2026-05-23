#include "shape_match/io/MatchReportWriter.h"

#include <nlohmann/json.hpp>

#include <fstream>
#include <iomanip>
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
        if (c == '"') {
            out += "\"\"";
        } else {
            out += c;
        }
    }
    out += '"';
    return out;
}

nlohmann::json poseJson(const MatchPose& p)
{
    return {
        {"x", p.x},
        {"y", p.y},
        {"theta_deg", p.thetaDeg()},
        {"scale", p.scale}
    };
}

nlohmann::json scoreJson(const MatchScore& s)
{
    return {
        {"finalScore", s.finalScore},
        {"coverageScore", s.coverageScore},
        {"distanceScore", s.distanceScore},
        {"orientationScore", s.orientationScore},
        {"polarityScore", s.polarityScore},
        {"inlierRatio", s.inlierRatio},
        {"coverageRatio", s.coverageRatio},
        {"validPoints", s.validPoints},
        {"foundPoints", s.foundPoints},
        {"inlierPoints", s.inlierPoints},
        {"polarityOkPoints", s.polarityOkPoints},
        {"rmsError", s.rmsError},
        {"meanError", s.meanError},
        {"medianError", s.medianError},
        {"p90Error", s.p90Error},
        {"maxError", s.maxError},
        {"meanAngleDiffDeg", s.meanAngleDiffDeg},
        {"medianAngleDiffDeg", s.medianAngleDiffDeg},
        {"accepted", s.accepted},
        {"rejectReason", s.rejectReason}
    };
}

nlohmann::json poseErrorJson(const PoseError& e)
{
    return {
        {"hasGroundTruth", e.hasGroundTruth},
        {"dx", e.dx},
        {"dy", e.dy},
        {"dxy", e.dxy},
        {"dtheta_deg", e.dthetaDeg},
        {"dscale", e.dscale},
        {"xyOk", e.xyOk},
        {"angleOk", e.angleOk},
        {"scaleOk", e.scaleOk},
        {"poseOk", e.poseOk}
    };
}

} // namespace

bool MatchReportWriter::writeJsonReport(const MatchDiagnosticReport& report,
                                        const std::filesystem::path& path) const
{
    std::filesystem::create_directories(path.parent_path());
    nlohmann::json root;
    root["imageName"] = report.imageName;
    root["templateName"] = report.templateName;
    root["candidateCount"] = report.candidateCount;
    root["coarseTimeMs"] = report.coarseTimeMs;
    root["refineTimeMs"] = report.refineTimeMs;
    root["evaluationTimeMs"] = report.evaluationTimeMs;
    root["totalTimeMs"] = report.totalTimeMs;
    root["pyramidLevel"] = report.pyramidLevel;
    root["failureReason"] = report.failureReason;
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

    for (const GroundTruthInstance& gt : report.groundTruthInstances) {
        root["groundTruthInstances"].push_back({{"id", gt.id}, {"pose", poseJson(gt.pose)}});
    }

    for (const ScoredCandidate& c : report.topCandidates) {
        root["topCandidates"].push_back({
            {"rank", c.rank},
            {"pose", poseJson(c.pose)},
            {"score", scoreJson(c.score)},
            {"poseError", poseErrorJson(c.poseError)},
            {"contourError", {
                {"meanError", c.contourError.meanError},
                {"rmsError", c.contourError.rmsError},
                {"medianError", c.contourError.medianError},
                {"p90Error", c.contourError.p90Error},
                {"maxError", c.contourError.maxError},
                {"pointCount", c.contourError.pointCount}
            }}
        });
    }

    std::ofstream file(path);
    if (!file) {
        return false;
    }
    file << root.dump(2);
    return file.good();
}

bool MatchReportWriter::writeCandidatesCsv(const MatchDiagnosticReport& report,
                                           const std::filesystem::path& path) const
{
    std::filesystem::create_directories(path.parent_path());
    std::ofstream file(path);
    if (!file) {
        return false;
    }
    file << "rank,x,y,theta_deg,scale,final_score,coverage_score,coverage_ratio,distance_score,"
            "orientation_score,polarity_score,inlier_ratio,rms_error,mean_error,median_error,p90_error,"
            "max_error,mean_angle_diff_deg,median_angle_diff_deg,valid_points,found_points,inlier_points,"
            "polarity_ok_points,accepted,reject_reason,has_ground_truth,dx,dy,dxy,dtheta_deg,dscale,pose_ok\n";
    file << std::fixed << std::setprecision(6);
    for (const ScoredCandidate& c : report.topCandidates) {
        const MatchScore& s = c.score;
        const PoseError& e = c.poseError;
        file << c.rank << ',' << c.pose.x << ',' << c.pose.y << ',' << c.pose.thetaDeg() << ',' << c.pose.scale << ','
             << s.finalScore << ',' << s.coverageScore << ',' << s.coverageRatio << ',' << s.distanceScore << ','
             << s.orientationScore << ',' << s.polarityScore << ',' << s.inlierRatio << ',' << s.rmsError << ','
             << s.meanError << ',' << s.medianError << ',' << s.p90Error << ',' << s.maxError << ','
             << s.meanAngleDiffDeg << ',' << s.medianAngleDiffDeg << ',' << s.validPoints << ',' << s.foundPoints << ','
             << s.inlierPoints << ',' << s.polarityOkPoints << ',' << (s.accepted ? "true" : "false") << ','
             << csvEscape(s.rejectReason) << ',' << (e.hasGroundTruth ? "true" : "false") << ','
             << e.dx << ',' << e.dy << ',' << e.dxy << ',' << e.dthetaDeg << ',' << e.dscale << ','
             << (e.poseOk ? "true" : "false") << '\n';
    }
    return file.good();
}

bool MatchReportWriter::writePointEvalCsv(const MatchDiagnosticReport& report,
                                          const std::filesystem::path& path) const
{
    std::filesystem::create_directories(path.parent_path());
    std::ofstream file(path);
    if (!file) {
        return false;
    }
    file << "candidate_rank,template_point_id,template_x,template_y,predicted_x,predicted_y,matched_x,matched_y,"
            "valid,found,inlier,polarity_ok,distance,normal_residual,tangent_residual,angle_diff_deg,gradient_mag,"
            "weight,distance_score,orientation_score,polarity_score,point_score,reject_reason\n";
    file << std::fixed << std::setprecision(6);
    for (const ScoredCandidate& c : report.topCandidates) {
        for (const PointEval& e : c.pointEvaluations) {
            file << c.rank << ',' << e.templatePointId << ',' << e.templatePt.x << ',' << e.templatePt.y << ','
                 << e.predictedPt.x << ',' << e.predictedPt.y << ',' << e.matchedEdgePt.x << ',' << e.matchedEdgePt.y << ','
                 << (e.valid ? "true" : "false") << ',' << (e.found ? "true" : "false") << ','
                 << (e.inlier ? "true" : "false") << ',' << (e.polarityOk ? "true" : "false") << ','
                 << e.distance << ',' << e.normalResidual << ',' << e.tangentResidual << ',' << e.angleDiffDeg << ','
                 << e.gradientMag << ',' << e.weight << ',' << e.distanceScore << ',' << e.orientationScore << ','
                 << e.polarityScore << ',' << e.pointScore << ',' << csvEscape(e.rejectReason) << '\n';
        }
    }
    return file.good();
}

bool MatchReportWriter::writeSummaryTxt(const MatchDiagnosticReport& report,
                                        const std::filesystem::path& path) const
{
    std::filesystem::create_directories(path.parent_path());
    std::ofstream file(path);
    if (!file) {
        return false;
    }
    file << std::fixed << std::setprecision(4);
    file << "imageName: " << report.imageName << "\n";
    file << "templateName: " << report.templateName << "\n";
    file << "candidateCount: " << report.candidateCount << "\n";
    if (!report.topCandidates.empty()) {
        const ScoredCandidate& top = report.topCandidates.front();
        file << "top1Pose: x=" << top.pose.x << " y=" << top.pose.y << " theta_deg=" << top.pose.thetaDeg()
             << " scale=" << top.pose.scale << "\n";
        file << "top1Score: " << top.score.finalScore << "\n";
        file << "top1Coverage: " << top.score.coverageRatio << "\n";
        file << "top1InlierRatio: " << top.score.inlierRatio << "\n";
        file << "top1Errors: rms=" << top.score.rmsError << " median=" << top.score.medianError
             << " p90=" << top.score.p90Error << "\n";
        if (top.poseError.hasGroundTruth) {
            file << "top1PoseError: dx=" << top.poseError.dx << " dy=" << top.poseError.dy
                 << " dxy=" << top.poseError.dxy << " dtheta_deg=" << top.poseError.dthetaDeg
                 << " poseOk=" << (top.poseError.poseOk ? "true" : "false") << "\n";
        }
    }
    file << "top1Hit: " << (report.multiTargetEval.top1Hit ? "true" : "false") << "\n";
    file << "top3Hit: " << (report.multiTargetEval.top3Hit ? "true" : "false") << "\n";
    file << "top5Hit: " << (report.multiTargetEval.top5Hit ? "true" : "false") << "\n";
    file << "top10Hit: " << (report.multiTargetEval.top10Hit ? "true" : "false") << "\n";
    file << "TP/FP/FN: " << report.multiTargetEval.truePositive << '/'
         << report.multiTargetEval.falsePositive << '/' << report.multiTargetEval.falseNegative << "\n";
    file << "Precision/Recall/F1: " << report.multiTargetEval.precision << '/'
         << report.multiTargetEval.recall << '/' << report.multiTargetEval.f1 << "\n";
    file << "evaluationTimeMs: " << report.evaluationTimeMs << "\n";
    if (!report.failureReason.empty()) {
        file << "failureReason: " << report.failureReason << "\n";
    }
    return file.good();
}

bool MatchReportWriter::writeAll(const MatchDiagnosticReport& report,
                                 const std::filesystem::path& reportsDir) const
{
    std::filesystem::create_directories(reportsDir);
    return writeJsonReport(report, reportsDir / "latest_match_report.json")
        && writeCandidatesCsv(report, reportsDir / "latest_candidates.csv")
        && writePointEvalCsv(report, reportsDir / "latest_point_eval.csv")
        && writeSummaryTxt(report, reportsDir / "latest_summary.txt");
}

} // namespace ShapeMatch
