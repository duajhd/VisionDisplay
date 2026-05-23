#include "shape_match/overlay/MatchDebugOverlayAdapter.h"

#include <iomanip>
#include <sstream>

namespace ShapeMatch {

namespace {

std::string candidateColor(int rank)
{
    if (rank == 1) {
        return "#ffd84d";
    }
    if (rank == 2) {
        return "#7fd7ff";
    }
    if (rank == 3) {
        return "#b67dff";
    }
    return "#999999";
}

} // namespace

ShapeMatchOverlayData MatchDebugOverlayAdapter::buildOverlayData(const MatchDiagnosticReport& report,
                                                                 const ShapeTemplateModel& model) const
{
    ShapeMatchOverlayData data;
    for (const ScoredCandidate& candidate : report.topCandidates) {
        ShapeMatchOverlayPolyline contour;
        contour.color = candidateColor(candidate.rank);
        contour.width = candidate.rank == 1 ? 2.5 : 1.2;
        contour.label = "rank " + std::to_string(candidate.rank);
        contour.points.reserve(model.points.size() + 1);
        for (const TemplatePoint& p : model.points) {
            contour.points.push_back(candidate.pose.transformPoint(p.position));
        }
        if (!contour.points.empty()) {
            contour.points.push_back(contour.points.front());
            data.polylines.push_back(std::move(contour));
        }

        if (candidate.rank == 1) {
            for (const PointEval& e : candidate.pointEvaluations) {
                ShapeMatchOverlayPoint point;
                point.pos = e.predictedPt;
                point.radius = 3.0;
                if (!e.found) {
                    point.color = "#ff4040";
                } else if (!e.polarityOk) {
                    point.color = "#ff8c00";
                } else if (e.inlier) {
                    point.color = "#00e060";
                } else {
                    point.color = "#d75cff";
                }
                data.points.push_back(point);

                if (e.found && e.distance > 0.2) {
                    ShapeMatchOverlayLine line;
                    line.p0 = e.predictedPt;
                    line.p1 = e.matchedEdgePt;
                    line.color = e.inlier ? "#40a0ff" : "#ff4040";
                    line.width = 1.0;
                    data.lines.push_back(line);
                }
            }
        }

        std::ostringstream text;
        text << "rank " << candidate.rank
             << " score=" << std::fixed << std::setprecision(3) << candidate.score.finalScore
             << " cov=" << candidate.score.coverageRatio
             << " inlier=" << candidate.score.inlierRatio
             << " rms=" << candidate.score.rmsError;
        ShapeMatchOverlayText label;
        label.pos = cv::Point2d(candidate.pose.x + 8.0, candidate.pose.y + 8.0 + 18.0 * (candidate.rank - 1));
        label.text = text.str();
        label.color = candidateColor(candidate.rank);
        label.fontSize = candidate.rank == 1 ? 15 : 12;
        data.texts.push_back(label);

        const double axisLen = candidate.rank == 1 ? 35.0 : 22.0;
        ShapeMatchOverlayLine xAxis;
        xAxis.p0 = cv::Point2d(candidate.pose.x, candidate.pose.y);
        xAxis.p1 = candidate.pose.transformPoint(cv::Point2d(axisLen, 0.0));
        xAxis.color = "#ff5050";
        xAxis.width = candidate.rank == 1 ? 2.0 : 1.0;
        data.lines.push_back(xAxis);

        ShapeMatchOverlayLine yAxis;
        yAxis.p0 = cv::Point2d(candidate.pose.x, candidate.pose.y);
        yAxis.p1 = candidate.pose.transformPoint(cv::Point2d(0.0, axisLen));
        yAxis.color = "#50ff50";
        yAxis.width = candidate.rank == 1 ? 2.0 : 1.0;
        data.lines.push_back(yAxis);
    }
    return data;
}

} // namespace ShapeMatch
