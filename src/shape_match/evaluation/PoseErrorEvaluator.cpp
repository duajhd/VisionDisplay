#include "shape_match/evaluation/PoseErrorEvaluator.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace ShapeMatch {

namespace {

double percentile(std::vector<double> values, double q)
{
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const double idx = std::clamp(q, 0.0, 1.0) * static_cast<double>(values.size() - 1);
    const size_t lo = static_cast<size_t>(std::floor(idx));
    const size_t hi = static_cast<size_t>(std::ceil(idx));
    const double f = idx - static_cast<double>(lo);
    return values[lo] * (1.0 - f) + values[hi] * f;
}

} // namespace

PoseErrorEvaluator::PoseErrorEvaluator(ShapeMatchEvalConfig config)
    : m_config(config)
{
}

PoseError PoseErrorEvaluator::evaluatePoseError(const MatchPose& predicted, const MatchPose& groundTruth) const
{
    PoseError e;
    e.hasGroundTruth = true;
    e.dx = predicted.x - groundTruth.x;
    e.dy = predicted.y - groundTruth.y;
    e.dxy = std::sqrt(e.dx * e.dx + e.dy * e.dy);
    e.dthetaRad = wrapToPi(predicted.theta - groundTruth.theta);
    e.dthetaDeg = std::abs(radToDeg(e.dthetaRad));
    e.dscale = predicted.scale - groundTruth.scale;
    e.xyOk = e.dxy <= m_config.xyOkThresholdPx;
    e.angleOk = e.dthetaDeg <= m_config.angleOkThresholdDeg;
    e.scaleOk = std::abs(e.dscale) <= m_config.scaleOkThreshold;
    e.poseOk = e.xyOk && e.angleOk && e.scaleOk;
    return e;
}

ContourReprojectionError PoseErrorEvaluator::evaluateContourReprojectionError(const ShapeTemplateModel& model,
                                                                              const MatchPose& predicted,
                                                                              const MatchPose& groundTruth) const
{
    ContourReprojectionError out;
    std::vector<double> errors;
    errors.reserve(model.points.size());
    for (const TemplatePoint& p : model.points) {
        const cv::Point2d pp = predicted.transformPoint(p.position);
        const cv::Point2d gp = groundTruth.transformPoint(p.position);
        errors.push_back(norm(pp - gp));
    }
    out.pointCount = static_cast<int>(errors.size());
    if (errors.empty()) {
        return out;
    }
    out.meanError = std::accumulate(errors.begin(), errors.end(), 0.0) / static_cast<double>(errors.size());
    const double sq = std::accumulate(errors.begin(), errors.end(), 0.0, [](double acc, double v) {
        return acc + v * v;
    });
    out.rmsError = std::sqrt(sq / static_cast<double>(errors.size()));
    out.medianError = percentile(errors, 0.50);
    out.p90Error = percentile(errors, 0.90);
    out.maxError = *std::max_element(errors.begin(), errors.end());
    return out;
}

} // namespace ShapeMatch
