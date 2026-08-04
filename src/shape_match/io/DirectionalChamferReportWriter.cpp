#include "shape_match/io/DirectionalChamferReportWriter.h"

#include "shape_match/core/ShapeMatchTypes.h"

#include <fstream>

#include <nlohmann/json.hpp>

namespace ShapeMatch {

DirectionalChamferReportWriter::DirectionalChamferReportWriter(std::filesystem::path reportDir)
    : m_reportDir(std::move(reportDir))
{
}

void DirectionalChamferReportWriter::write(const DirectionalChamferDebugReport& report) const
{
    std::filesystem::create_directories(m_reportDir);
    {
        nlohmann::json j;
        j["input_candidates"] = report.inputCandidates;
        j["accepted_candidates"] = report.acceptedCandidates;
        j["pruned_candidates"] = report.prunedCandidates;
        j["final_candidates_to_ranker"] = report.finalCandidatesToRanker;
        j["field_build_time_ms"] = report.fieldBuildTimeMs;
        j["verify_time_ms"] = report.verifyTimeMs;
        j["avg_time_per_candidate_ms"] = report.avgTimePerCandidateMs;
        j["top_score"] = report.topScore;
        j["used_old_local_refine"] = report.usedOldLocalRefine;
        std::ofstream out(m_reportDir / "latest_directional_chamfer_profile.json");
        if (out.is_open()) {
            out << j.dump(2);
        }
    }
    {
        std::ofstream out(m_reportDir / "latest_directional_chamfer_candidates.csv");
        out << "rank,x,y,theta_deg,response_score,fast_score,directional_chamfer_score,distance_score,orientation_score,coverage_score,active_segments,total_segments,accepted,pruned,reject_reason\n";
        for (size_t i = 0; i < report.candidates.size(); ++i) {
            const auto& row = report.candidates[i];
            out << (i + 1) << ','
                << row.candidate.pose.x << ','
                << row.candidate.pose.y << ','
                << row.candidate.pose.thetaDeg() << ','
                << row.candidate.responseScore << ','
                << row.candidate.fastScore << ','
                << row.score.finalScore << ','
                << row.score.distanceScore << ','
                << row.score.orientationScore << ','
                << row.score.coverageScore << ','
                << row.score.activeSegments << ','
                << row.score.totalSegments << ','
                << row.score.accepted << ','
                << row.score.pruned << ','
                << row.score.rejectReason << '\n';
        }
    }
    {
        std::ofstream out(m_reportDir / "latest_directional_chamfer_segments.csv");
        out << "candidate_rank,segment_id,total_points,evaluated_points,matched_points,coverage,mean_distance,mean_orientation_error_deg,distance_score,orientation_score,segment_score,active\n";
        for (size_t i = 0; i < report.candidates.size(); ++i) {
            const auto& row = report.candidates[i];
            for (const SegmentScore& seg : row.score.segments) {
                out << (i + 1) << ','
                    << seg.segmentId << ','
                    << seg.totalPoints << ','
                    << seg.evaluatedPoints << ','
                    << seg.matchedPoints << ','
                    << seg.coverage << ','
                    << seg.meanDistance << ','
                    << seg.meanOrientationErrorDeg << ','
                    << seg.distanceScore << ','
                    << seg.orientationScore << ','
                    << seg.segmentScore << ','
                    << seg.active << '\n';
            }
        }
    }
    {
        std::ofstream out(m_reportDir / "latest_directional_chamfer_summary.txt");
        out << "Directional Chamfer Summary\n";
        out << "inputCandidates: " << report.inputCandidates << '\n';
        out << "acceptedCandidates: " << report.acceptedCandidates << '\n';
        out << "prunedCandidates: " << report.prunedCandidates << '\n';
        out << "finalCandidatesToRanker: " << report.finalCandidatesToRanker << '\n';
        out << "fieldBuildTimeMs: " << report.fieldBuildTimeMs << '\n';
        out << "verifyTimeMs: " << report.verifyTimeMs << '\n';
        out << "avgTimePerCandidateMs: " << report.avgTimePerCandidateMs << '\n';
        out << "topScore: " << report.topScore << '\n';
        out << "usedOldLocalRefine: " << (report.usedOldLocalRefine ? "true" : "false") << '\n';
    }
}

} // namespace ShapeMatch
