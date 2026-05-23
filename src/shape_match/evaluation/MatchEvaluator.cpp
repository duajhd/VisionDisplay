#include "shape_match/evaluation/MatchEvaluator.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace ShapeMatch {

namespace {

bool validPoint(const cv::Point2d& p)
{
    return std::isfinite(p.x) && std::isfinite(p.y);
}

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

double meanOf(const std::vector<double>& values)
{
    if (values.empty()) {
        return 0.0;
    }
    return std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
}

double weightedMean(const std::vector<PointEval>& evals, double PointEval::*member)
{
    double sum = 0.0;
    double weightSum = 0.0;
    for (const PointEval& e : evals) {
        if (!e.found) {
            continue;
        }
        sum += e.*member * e.weight;
        weightSum += e.weight;
    }
    return weightSum > 0.0 ? sum / weightSum : 0.0;
}

} // namespace

MatchEvaluator::MatchEvaluator(ShapeMatchEvalConfig config)
    : m_config(config)
{
}

MatchScore MatchEvaluator::evaluate(const ShapeTemplateModel& model,
                                    const EdgeImageData& edgeData,
                                    const MatchPose& pose,
                                    std::vector<PointEval>* pointEvals,
                                    MatchEvaluationStats* stats) const
{
    MatchEvaluationStats localStats;
    MatchEvaluationStats* activeStats = stats ? stats : &localStats;
    activeStats->evaluationMode = edgeData.hasNearestEdgeField() && m_config.enableDistanceFieldEvaluation
        ? "distance_field"
        : "local_window";

    std::vector<PointEval> evals;
    evals.reserve(model.points.size());
    for (const TemplatePoint& pt : model.points) {
        evals.push_back(evaluatePoint(pt, edgeData, pose, activeStats));
    }

    MatchScore score;
    computeAggregateStats(score, evals);
    determineAccepted(score);
    if (pointEvals) {
        *pointEvals = std::move(evals);
    }
    return score;
}

MatchScore MatchEvaluator::evaluate(const ShapeTemplateModel& model,
                                    const EdgeQueryContext& queryContext,
                                    const MatchPose& pose,
                                    std::vector<PointEval>* pointEvals,
                                    MatchEvaluationStats* stats) const
{
    MatchEvaluationStats localStats;
    MatchEvaluationStats* activeStats = stats ? stats : &localStats;
    activeStats->evaluationMode = queryContext.roiFields ? "roi_distance_field" : "full_distance_field";

    std::vector<PointEval> evals;
    evals.reserve(model.points.size());
    for (const TemplatePoint& pt : model.points) {
        evals.push_back(evaluatePoint(pt, queryContext, pose, activeStats));
    }

    MatchScore score;
    computeAggregateStats(score, evals);
    determineAccepted(score);
    if (pointEvals) {
        *pointEvals = std::move(evals);
    }
    return score;
}

PointEval MatchEvaluator::evaluatePoint(const TemplatePoint& pt,
                                        const EdgeImageData& edgeData,
                                        const MatchPose& pose,
                                        MatchEvaluationStats* stats) const
{
    PointEval e;
    e.templatePointId = pt.id;
    e.templatePt = pt.position;
    e.weight = std::max(0.0, pt.weight);
    e.valid = validPoint(pt.position) && e.weight > 0.0;
    if (!e.valid) {
        e.rejectReason = "invalid_template_point";
        return e;
    }

    e.predictedPt = pose.transformPoint(pt.position);
    if (!edgeData.isInside(e.predictedPt)) {
        e.rejectReason = "predicted_outside_image";
        return e;
    }

    cv::Point2d imageNormal;
    bool found = false;
    if (m_config.enableDistanceFieldEvaluation && m_config.enableNearestEdgeFieldEvaluation) {
        found = edgeData.findNearestEdgeFast(e.predictedPt,
                                             m_config.searchRadiusPx,
                                             e.matchedEdgePt,
                                             imageNormal,
                                             e.gradientMag,
                                             e.distance);
        if (found && stats) {
            ++stats->distanceFieldQueryCount;
            stats->evaluationMode = "distance_field";
        }
    }
    if (!found && m_config.allowLocalSearchFallbackInEvaluator) {
        found = edgeData.findNearestEdgeLocalWindow(e.predictedPt,
                                                   std::max<double>(1.0, m_config.evaluatorLocalSearchRadiusPx),
                                                   e.matchedEdgePt,
                                                   imageNormal,
                                                   e.gradientMag,
                                                   e.distance);
        if (found && stats) {
            ++stats->localWindowFallbackCount;
            if (stats->evaluationMode != "distance_field") {
                stats->evaluationMode = "local_window";
            }
        }
    }
    if (!found) {
        const long long pixelCount = static_cast<long long>(edgeData.imageSize.width) * static_cast<long long>(edgeData.imageSize.height);
        if (pixelCount <= 1000000LL || edgeData.edgePoints.size() <= 5000) {
            found = edgeData.findNearestEdgeLinearScanDebugOnly(e.predictedPt,
                                                               m_config.searchRadiusPx,
                                                               e.matchedEdgePt,
                                                               imageNormal,
                                                               e.gradientMag,
                                                               e.distance);
            if (found && stats) {
                ++stats->linearScanFallbackCount;
                stats->evaluationMode = "linear_scan_debug_only";
            }
        }
    }
    if (!found) {
        if (stats) {
            ++stats->failedLookupCount;
        }
        e.rejectReason = "edge_not_found";
        return e;
    }

    e.found = true;
    const cv::Point2d templateNormal = norm(pt.normal) > 1e-6 ? pt.normal : pt.gradientDir;
    const cv::Point2d predNormal = normalized(pose.rotateVector(templateNormal));
    const cv::Point2d predTangent = normalized(pose.rotateVector(norm(pt.tangent) > 1e-6
                                                                     ? pt.tangent
                                                                     : cv::Point2d(-templateNormal.y, templateNormal.x)));
    imageNormal = normalized(imageNormal);

    const cv::Point2d residual = e.matchedEdgePt - e.predictedPt;
    e.normalResidual = residual.x * predNormal.x + residual.y * predNormal.y;
    e.tangentResidual = residual.x * predTangent.x + residual.y * predTangent.y;
    e.angleDiffRad = angleBetweenUnitVectors(predNormal, imageNormal);
    e.angleDiffDeg = radToDeg(e.angleDiffRad);

    const double normalDot = predNormal.x * imageNormal.x + predNormal.y * imageNormal.y;
    if (pt.polarity == EdgePolarity::Any || !m_config.enablePolarityCheck) {
        e.polarityOk = true;
    } else if (pt.polarity == EdgePolarity::DarkToBright) {
        e.polarityOk = normalDot >= 0.0;
    } else {
        e.polarityOk = normalDot <= 0.0;
    }

    e.distanceScore = computeDistanceScore(e.distance, m_config.coarseDistanceSigma);
    e.orientationScore = computeOrientationScore(e.angleDiffDeg);
    e.polarityScore = computePolarityScore(e.polarityOk, pt.polarity);
    e.inlier = e.distance <= m_config.inlierDistanceThresholdPx
        && e.angleDiffDeg <= m_config.inlierAngleThresholdDeg
        && e.polarityScore > 0.5;
    e.pointScore = m_config.wPointDistance * e.distanceScore
        + m_config.wPointOrientation * e.orientationScore
        + m_config.wPointPolarity * e.polarityScore;
    if (!e.inlier) {
        e.rejectReason = "not_inlier";
    }
    return e;
}

PointEval MatchEvaluator::evaluatePoint(const TemplatePoint& pt,
                                        const EdgeQueryContext& queryContext,
                                        const MatchPose& pose,
                                        MatchEvaluationStats* stats) const
{
    PointEval e;
    e.templatePointId = pt.id;
    e.templatePt = pt.position;
    e.weight = std::max(0.0, pt.weight);
    e.valid = validPoint(pt.position) && e.weight > 0.0;
    if (!e.valid) {
        e.rejectReason = "invalid_template_point";
        return e;
    }
    if (!queryContext.fullEdgeData) {
        e.rejectReason = "missing_edge_data";
        return e;
    }

    e.predictedPt = pose.transformPoint(pt.position);
    if (!queryContext.fullEdgeData->isInside(e.predictedPt)) {
        e.rejectReason = "predicted_outside_image";
        return e;
    }

    cv::Point2d imageNormal;
    std::string mode;
    if (!queryNearestEdge(queryContext,
                          0,
                          e.predictedPt,
                          m_config.searchRadiusPx,
                          e.matchedEdgePt,
                          imageNormal,
                          e.gradientMag,
                          e.distance,
                          &mode)) {
        if (stats) {
            ++stats->failedLookupCount;
            ++stats->missingFieldCount;
        }
        e.rejectReason = "edge_not_found";
        return e;
    }
    if (stats) {
        if (mode == "roi_distance_field") {
            ++stats->roiDistanceFieldQueryCount;
            stats->evaluationMode = "roi_distance_field";
        } else if (mode == "full_distance_field") {
            ++stats->fullDistanceFieldQueryCount;
            if (stats->evaluationMode != "roi_distance_field") {
                stats->evaluationMode = "full_distance_field";
            }
        } else if (mode == "local_window") {
            ++stats->localWindowFallbackCount;
            if (stats->evaluationMode != "roi_distance_field" && stats->evaluationMode != "full_distance_field") {
                stats->evaluationMode = "local_window";
            }
        }
    }

    e.found = true;
    const cv::Point2d templateNormal = norm(pt.normal) > 1e-6 ? pt.normal : pt.gradientDir;
    const cv::Point2d predNormal = normalized(pose.rotateVector(templateNormal));
    const cv::Point2d predTangent = normalized(pose.rotateVector(norm(pt.tangent) > 1e-6
                                                                     ? pt.tangent
                                                                     : cv::Point2d(-templateNormal.y, templateNormal.x)));
    imageNormal = normalized(imageNormal);

    const cv::Point2d residual = e.matchedEdgePt - e.predictedPt;
    e.normalResidual = residual.x * predNormal.x + residual.y * predNormal.y;
    e.tangentResidual = residual.x * predTangent.x + residual.y * predTangent.y;
    e.angleDiffRad = angleBetweenUnitVectors(predNormal, imageNormal);
    e.angleDiffDeg = radToDeg(e.angleDiffRad);

    const double normalDot = predNormal.x * imageNormal.x + predNormal.y * imageNormal.y;
    if (pt.polarity == EdgePolarity::Any || !m_config.enablePolarityCheck) {
        e.polarityOk = true;
    } else if (pt.polarity == EdgePolarity::DarkToBright) {
        e.polarityOk = normalDot >= 0.0;
    } else {
        e.polarityOk = normalDot <= 0.0;
    }

    e.distanceScore = computeDistanceScore(e.distance, m_config.coarseDistanceSigma);
    e.orientationScore = computeOrientationScore(e.angleDiffDeg);
    e.polarityScore = computePolarityScore(e.polarityOk, pt.polarity);
    e.inlier = e.distance <= m_config.inlierDistanceThresholdPx
        && e.angleDiffDeg <= m_config.inlierAngleThresholdDeg
        && e.polarityScore > 0.5;
    e.pointScore = m_config.wPointDistance * e.distanceScore
        + m_config.wPointOrientation * e.orientationScore
        + m_config.wPointPolarity * e.polarityScore;
    if (!e.inlier) {
        e.rejectReason = "not_inlier";
    }
    return e;
}

double MatchEvaluator::computeDistanceScore(double distance, double sigma) const
{
    sigma = std::max(1e-6, sigma);
    return std::exp(-(distance * distance) / (2.0 * sigma * sigma));
}

double MatchEvaluator::computeOrientationScore(double angleDiffDeg) const
{
    if (!m_config.enableOrientationCheck) {
        return 1.0;
    }
    const double sigma = std::max(1e-6, m_config.angleSigmaDeg);
    return std::exp(-(angleDiffDeg * angleDiffDeg) / (2.0 * sigma * sigma));
}

double MatchEvaluator::computePolarityScore(bool polarityOk, EdgePolarity polarity) const
{
    if (!m_config.enablePolarityCheck || polarity == EdgePolarity::Any) {
        return 1.0;
    }
    return polarityOk ? 1.0 : 0.0;
}

void MatchEvaluator::computeAggregateStats(MatchScore& score, const std::vector<PointEval>& evals) const
{
    score.totalPoints = static_cast<int>(evals.size());
    std::vector<double> errors;
    std::vector<double> angles;
    errors.reserve(evals.size());
    angles.reserve(evals.size());

    for (const PointEval& e : evals) {
        if (e.valid) {
            ++score.validPoints;
        }
        if (e.found) {
            ++score.foundPoints;
            errors.push_back(e.distance);
            angles.push_back(e.angleDiffDeg);
        }
        if (e.inlier) {
            ++score.inlierPoints;
        }
        if (e.found && e.polarityOk) {
            ++score.polarityOkPoints;
        }
    }

    if (score.validPoints > 0) {
        score.coverageRatio = static_cast<double>(score.foundPoints) / static_cast<double>(score.validPoints);
        score.inlierRatio = static_cast<double>(score.inlierPoints) / static_cast<double>(score.validPoints);
    }
    score.coverageScore = score.coverageRatio;
    score.distanceScore = weightedMean(evals, &PointEval::distanceScore);
    score.orientationScore = weightedMean(evals, &PointEval::orientationScore);
    score.polarityScore = score.foundPoints > 0
        ? static_cast<double>(score.polarityOkPoints) / static_cast<double>(score.foundPoints)
        : 0.0;

    if (!errors.empty()) {
        score.meanError = meanOf(errors);
        const double sq = std::accumulate(errors.begin(), errors.end(), 0.0, [](double acc, double v) {
            return acc + v * v;
        });
        score.rmsError = std::sqrt(sq / static_cast<double>(errors.size()));
        score.medianError = percentile(errors, 0.50);
        score.p75Error = percentile(errors, 0.75);
        score.p90Error = percentile(errors, 0.90);
        score.p95Error = percentile(errors, 0.95);
        score.maxError = *std::max_element(errors.begin(), errors.end());
    }

    if (!angles.empty()) {
        score.meanAngleDiffDeg = meanOf(angles);
        score.medianAngleDiffDeg = percentile(angles, 0.50);
        score.p90AngleDiffDeg = percentile(angles, 0.90);
    }

    score.finalScore = m_config.wCoverage * score.coverageScore
        + m_config.wDistance * score.distanceScore
        + m_config.wOrientation * score.orientationScore
        + m_config.wPolarity * score.polarityScore
        + m_config.wInlier * score.inlierRatio
        - score.ambiguityPenalty
        - score.occlusionPenalty;
    score.finalScore = std::max(0.0, score.finalScore);
}

bool MatchEvaluator::determineAccepted(MatchScore& score) const
{
    if (score.validPoints < m_config.minValidPoints) {
        score.accepted = false;
        score.rejectReason = "too_few_valid_points";
    } else if (score.coverageRatio < m_config.minCoverageRatio) {
        score.accepted = false;
        score.rejectReason = "low_coverage";
    } else if (score.inlierRatio < m_config.minInlierRatio) {
        score.accepted = false;
        score.rejectReason = "low_inlier_ratio";
    } else if (score.medianError > m_config.maxMedianErrorPx) {
        score.accepted = false;
        score.rejectReason = "high_median_error";
    } else if (score.p90Error > m_config.maxP90ErrorPx) {
        score.accepted = false;
        score.rejectReason = "high_p90_error";
    } else {
        score.accepted = true;
        score.rejectReason.clear();
    }
    return score.accepted;
}

} // namespace ShapeMatch
