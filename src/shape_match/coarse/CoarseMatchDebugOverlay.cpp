#include "shape_match/coarse/CoarseMatchDebugOverlay.h"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace ShapeMatch {

namespace {

std::string colorForRank(int rank)
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
    return "#8a8f98";
}

std::vector<cv::Point2d> transformedRect(const cv::Rect2d& rect, const MatchPose& pose)
{
    const std::vector<cv::Point2d> corners{
        cv::Point2d(rect.x, rect.y),
        cv::Point2d(rect.x + rect.width, rect.y),
        cv::Point2d(rect.x + rect.width, rect.y + rect.height),
        cv::Point2d(rect.x, rect.y + rect.height)
    };
    std::vector<cv::Point2d> out;
    out.reserve(5);
    for (const cv::Point2d& p : corners) {
        out.push_back(pose.transformPoint(p));
    }
    if (!out.empty()) {
        out.push_back(out.front());
    }
    return out;
}

const CoarseCandidate* findCoarseByPose(const std::vector<CoarseCandidate>& candidates, const MatchPose& pose)
{
    const CoarseCandidate* best = nullptr;
    double bestCost = 1e100;
    for (const CoarseCandidate& c : candidates) {
        const double dx = c.pose.x - pose.x;
        const double dy = c.pose.y - pose.y;
        const double da = std::abs(radToDeg(wrapToPi(c.pose.theta - pose.theta)));
        const double cost = std::sqrt(dx * dx + dy * dy) + da * 0.25;
        if (cost < bestCost) {
            bestCost = cost;
            best = &c;
        }
    }
    return best;
}

} // namespace

ShapeMatchOverlayData CoarseMatchDebugOverlay::buildOverlayData(const CoarseMatchReport& report,
                                                                const ShapeTemplateModel& model,
                                                                int maxCandidates,
                                                                bool includeCoarseSeedMarkers) const
{
    ShapeMatchOverlayData data;
    int count = 0;
    for (const ScoredCandidate& candidate : report.finalRankedCandidates) {
        if (++count > maxCandidates) {
            break;
        }
        const std::string color = colorForRank(candidate.rank);
        ShapeMatchOverlayPolyline contour;
        contour.color = color;
        contour.width = candidate.rank == 1 ? 2.8 : 1.2;
        contour.label = "coarse rank " + std::to_string(candidate.rank);
        contour.points.reserve(model.points.size() + 1);
        for (const TemplatePoint& p : model.points) {
            contour.points.push_back(candidate.pose.transformPoint(p.position));
        }
        if (!contour.points.empty()) {
            contour.points.push_back(contour.points.front());
            data.polylines.push_back(std::move(contour));
        }

        if (model.boundingBox.width > 0.0 && model.boundingBox.height > 0.0) {
            ShapeMatchOverlayPolyline roi;
            roi.color = color;
            roi.width = candidate.rank == 1 ? 2.4 : 1.0;
            roi.label = "ROI " + std::to_string(candidate.rank);
            roi.points = transformedRect(model.boundingBox, candidate.pose);
            data.polylines.push_back(std::move(roi));
        }

        ShapeMatchOverlayPoint center;
        center.pos = cv::Point2d(candidate.pose.x, candidate.pose.y);
        center.color = color;
        center.radius = candidate.rank == 1 ? 5.0 : 3.0;
        center.label = std::to_string(candidate.rank);
        data.points.push_back(center);

        const double axisLen = candidate.rank == 1 ? 35.0 : 22.0;
        ShapeMatchOverlayLine xAxis;
        xAxis.p0 = center.pos;
        xAxis.p1 = candidate.pose.transformPoint(cv::Point2d(axisLen, 0.0));
        xAxis.color = "#ff5050";
        xAxis.width = candidate.rank == 1 ? 2.0 : 1.0;
        data.lines.push_back(xAxis);

        ShapeMatchOverlayLine yAxis;
        yAxis.p0 = center.pos;
        yAxis.p1 = candidate.pose.transformPoint(cv::Point2d(0.0, axisLen));
        yAxis.color = "#50ff50";
        yAxis.width = candidate.rank == 1 ? 2.0 : 1.0;
        data.lines.push_back(yAxis);

        std::ostringstream text;
        text << "rank " << candidate.rank
             << " score=" << std::fixed << std::setprecision(3) << candidate.score.finalScore
             << " cov=" << candidate.score.coverageRatio
             << " inlier=" << candidate.score.inlierRatio
             << " rms=" << candidate.score.rmsError;
        if (const CoarseCandidate* coarse = findCoarseByPose(report.finalCoarseCandidates, candidate.pose)) {
            text << " src=" << coarse->source
                 << " fast=" << coarse->fastScore;
            if (coarse->voteScore > 0.0) {
                text << " vote=" << coarse->voteScore;
            }
        }
        ShapeMatchOverlayText label;
        label.pos = cv::Point2d(candidate.pose.x + 8.0, candidate.pose.y + 12.0 + candidate.rank * 14.0);
        label.text = text.str();
        label.color = color;
        label.fontSize = candidate.rank == 1 ? 15 : 12;
        data.texts.push_back(label);
    }

    if (includeCoarseSeedMarkers && !report.levels.empty()) {
        const CoarseMatchLevelResult& coarsest = report.levels.front();
        for (size_t i = 0; i < coarsest.topCandidates.size() && i < 20; ++i) {
            const CoarseCandidate& c = coarsest.topCandidates[i];
            const double scaleToBase = 1.0 / std::max(1e-12, std::pow(0.5, c.pyramidLevel));
            ShapeMatchOverlayPoint p;
            p.pos = cv::Point2d(c.pose.x * scaleToBase, c.pose.y * scaleToBase);
            p.color = c.source == "voting" ? "#34d399" : "#5ac8fa";
            p.radius = c.source == "voting" ? 3.0 : 2.0;
            p.label = c.source == "voting" ? "v" : "g";
            data.points.push_back(p);

            if (i < 5 && model.boundingBox.width > 0.0 && model.boundingBox.height > 0.0) {
                MatchPose basePose = c.pose;
                basePose.x *= scaleToBase;
                basePose.y *= scaleToBase;
                ShapeMatchOverlayPolyline projection;
                projection.color = c.source == "voting" ? "#34d399" : "#5ac8fa";
                projection.width = 0.9;
                projection.label = c.source + " candidate";
                projection.points = transformedRect(model.boundingBox, basePose);
                data.polylines.push_back(std::move(projection));

                std::ostringstream labelText;
                labelText << c.source
                          << " vote=" << std::fixed << std::setprecision(2) << c.voteScore
                          << " fast=" << c.fastScore;
                ShapeMatchOverlayText label;
                label.pos = cv::Point2d(basePose.x + 6.0, basePose.y - 10.0 - static_cast<double>(i) * 10.0);
                label.text = labelText.str();
                label.color = projection.color;
                label.fontSize = 11;
                data.texts.push_back(label);
            }
        }
    }
    return data;
}

} // namespace ShapeMatch
