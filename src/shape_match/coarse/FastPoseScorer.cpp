#include "shape_match/coarse/FastPoseScorer.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace ShapeMatch {

namespace {

float sampleFloat(const cv::Mat& mat, int x, int y, float fallback = 0.0f)
{
    if (mat.empty() || x < 0 || y < 0 || x >= mat.cols || y >= mat.rows) {
        return fallback;
    }
    return mat.at<float>(y, x);
}

int sampleInt(const cv::Mat& mat, int x, int y, int fallback = -1)
{
    if (mat.empty() || x < 0 || y < 0 || x >= mat.cols || y >= mat.rows) {
        return fallback;
    }
    return mat.at<int>(y, x);
}

bool fastNearestEdge(const EdgeImageData& image,
                     double px,
                     double py,
                     double searchRadius,
                     cv::Point2d& imageNormal,
                     double& gradientMag,
                     double& distance)
{
    if (image.edgeMap.empty() || px < 0.0 || py < 0.0 || px >= image.imageSize.width || py >= image.imageSize.height) {
        return false;
    }
    const int r = std::max(1, static_cast<int>(std::ceil(searchRadius)));
    const int cx = static_cast<int>(std::round(px));
    const int cy = static_cast<int>(std::round(py));
    double bestD2 = searchRadius * searchRadius;
    int bestX = -1;
    int bestY = -1;
    for (int y = std::max(0, cy - r); y <= std::min(image.edgeMap.rows - 1, cy + r); ++y) {
        const uchar* row = image.edgeMap.ptr<uchar>(y);
        for (int x = std::max(0, cx - r); x <= std::min(image.edgeMap.cols - 1, cx + r); ++x) {
            if (row[x] == 0) {
                continue;
            }
            const double dx = static_cast<double>(x) - px;
            const double dy = static_cast<double>(y) - py;
            const double d2 = dx * dx + dy * dy;
            if (d2 <= bestD2) {
                bestD2 = d2;
                bestX = x;
                bestY = y;
            }
        }
    }
    if (bestX < 0) {
        return false;
    }
    distance = std::sqrt(bestD2);
    const cv::Point2d g(sampleFloat(image.gradX, bestX, bestY), sampleFloat(image.gradY, bestX, bestY));
    imageNormal = normalized(g);
    gradientMag = sampleFloat(image.gradMag, bestX, bestY, 0.0f);
    return true;
}

double pointWeight(const TemplatePoint& pt)
{
    return std::max(0.01, pt.weight) * std::max(1.0, pt.gradientMag / 255.0);
}

bool shouldPrune(const CoarseMatchConfig& config,
                 const FastScoreContext& context,
                 int evaluated,
                 double scoreSum,
                 double remainingWeight,
                 double totalWeight,
                 CoarseCandidate& out)
{
    if (!config.enableGreedyUpperBoundPruning || !context.hasTopKThreshold || totalWeight <= 0.0) {
        return false;
    }
    if (evaluated < std::max(1, config.minEvaluatedPointsBeforePruning)) {
        return false;
    }
    const int interval = std::max(1, config.upperBoundCheckInterval);
    if ((evaluated % interval) != 0) {
        return false;
    }
    const double maxPossibleScore = (scoreSum + remainingWeight) / totalWeight;
    if (maxPossibleScore < static_cast<double>(context.currentTopKMinScore - config.upperBoundMargin)) {
        out.fastScore = maxPossibleScore;
        out.rejectedByEarlyExit = true;
        out.rejectReason = "upper_bound_pruned";
        return true;
    }
    return false;
}

} // namespace

FastPoseScorer::FastPoseScorer(CoarseMatchConfig config)
    : m_config(std::move(config))
{
}

CoarseCandidate FastPoseScorer::scorePose(const ShapeTemplateModel& templateLevel,
                                          const EdgeImageData& imageLevel,
                                          const MatchPose& pose,
                                          int pyramidLevel,
                                          double pruningScoreFloor) const
{
    FastScoreContext context;
    context.currentTopKMinScore = static_cast<float>(pruningScoreFloor);
    context.hasTopKThreshold = pruningScoreFloor >= 0.0;
    return scorePose(templateLevel, imageLevel, pose, pyramidLevel, context);
}

CoarseCandidate FastPoseScorer::scorePose(const ShapeTemplateModel& templateLevel,
                                          const EdgeImageData& imageLevel,
                                          const MatchPose& pose,
                                          int pyramidLevel,
                                          const FastScoreContext& context) const
{
    if (m_config.enableDistanceFieldScoring && imageLevel.hasDistanceField && !imageLevel.distanceMap.empty()) {
        return scorePoseDistanceField(templateLevel, imageLevel, pose, pyramidLevel, context);
    }
    return scorePoseLocalSearch(templateLevel, imageLevel, pose, pyramidLevel, context);
}

CoarseCandidate FastPoseScorer::scorePoseFast(const TemplatePointSoA& points,
                                              const RotatedTemplateCache& rotationCache,
                                              const EdgeImageData& imageLevel,
                                              const MatchPose& pose,
                                              int pyramidLevel,
                                              const FastScoreContext& context) const
{
    CoarseCandidate out;
    out.pose = pose;
    out.pyramidLevel = pyramidLevel;
    out.totalPointCount = points.size();

    if (points.size() < m_config.minTemplatePointsForScore || imageLevel.empty()) {
        out.rejectReason = "insufficient_input";
        return out;
    }
    if (!m_config.enableDistanceFieldScoring || !imageLevel.hasDistanceField || imageLevel.distanceMap.empty()) {
        out.rejectReason = "distance_field_unavailable";
        return out;
    }

    const RotatedTemplateView* rotated = m_config.enableRotatedTemplateCache
        ? rotationCache.findNearest(static_cast<float>(pose.theta))
        : nullptr;
    const float c = rotated ? rotated->cosTheta : std::cos(static_cast<float>(pose.theta));
    const float s = rotated ? rotated->sinTheta : std::sin(static_cast<float>(pose.theta));
    const float totalWeight = points.totalWeight > 0.0f ? points.totalWeight : static_cast<float>(std::max(1, points.size()));
    float scoreSum = 0.0f;
    float remainingWeight = totalWeight;
    double distanceSum = 0.0;
    double orientationSum = 0.0;
    double polaritySum = 0.0;
    const float maxDistance = std::max(0.1f, m_config.maxDistanceForScore);
    const std::vector<int>* order = points.orderedIndices.empty() ? nullptr : &points.orderedIndices;

    const auto scoreIndex = [&](int index) {
        const size_t i = static_cast<size_t>(index);
        const float weight = templatePointWeight(points.weight[i], points.gradMag[i]);
        remainingWeight = std::max(0.0f, remainingWeight - weight);
        ++out.evaluatedPoints;

        const float rx = rotated ? rotated->rx[i] : (points.x[i] * c - points.y[i] * s);
        const float ry = rotated ? rotated->ry[i] : (points.x[i] * s + points.y[i] * c);
        const float px = static_cast<float>(pose.x) + static_cast<float>(pose.scale) * rx;
        const float py = static_cast<float>(pose.y) + static_cast<float>(pose.scale) * ry;
        float pointScore = 0.0f;
        if (px >= 0.0f && py >= 0.0f && px < imageLevel.imageSize.width && py < imageLevel.imageSize.height) {
            const int ix = std::clamp(static_cast<int>(std::round(px)), 0, imageLevel.imageSize.width - 1);
            const int iy = std::clamp(static_cast<int>(std::round(py)), 0, imageLevel.imageSize.height - 1);
            const float d = sampleFloat(imageLevel.distanceMap, ix, iy, maxDistance + 1.0f);
            const float dScore = d <= maxDistance ? static_cast<float>(distanceScore(d)) : 0.0f;
            const float imageAngle = m_config.enableOrientationField && !imageLevel.orientationMap.empty()
                ? sampleFloat(imageLevel.orientationMap, ix, iy, 0.0f)
                : 0.0f;
            const float imgX = std::cos(imageAngle);
            const float imgY = std::sin(imageAngle);
            float predX = rotated ? rotated->rgx[i] : (points.gx[i] * c - points.gy[i] * s);
            float predY = rotated ? rotated->rgy[i] : (points.gx[i] * s + points.gy[i] * c);
            const float predNorm = std::sqrt(predX * predX + predY * predY);
            if (predNorm > 1e-6f) {
                predX /= predNorm;
                predY /= predNorm;
            }
            const float dot = std::clamp(predX * imgX + predY * imgY, -1.0f, 1.0f);
            const float angleDiffDeg = static_cast<float>(radToDeg(std::acos(dot)));
            const float oScore = m_config.useGradientOrientation ? static_cast<float>(orientationScore(angleDiffDeg)) : 1.0f;
            bool polarityOk = true;
            const EdgePolarity polarity = static_cast<EdgePolarity>(points.polarity[i]);
            if (polarity == EdgePolarity::DarkToBright) {
                polarityOk = dot >= 0.0f;
            } else if (polarity == EdgePolarity::BrightToDark) {
                polarityOk = dot <= 0.0f;
            }
            const float pScore = m_config.usePolarity ? static_cast<float>(polarityScore(polarityOk, polarity)) : 1.0f;
            if (d <= maxDistance) {
                ++out.matchedPoints;
            }
            distanceSum += dScore;
            orientationSum += oScore;
            polaritySum += pScore;
            pointScore = m_config.fastWDistance * dScore
                + m_config.fastWOrientation * oScore
                + m_config.fastWPolarity * pScore;
        }

        scoreSum += pointScore * weight;
        return shouldPrune(m_config, context, out.evaluatedPoints, scoreSum, remainingWeight, totalWeight, out);
    };

    if (order) {
        for (int index : *order) {
            if (scoreIndex(index)) {
                break;
            }
        }
    } else {
        for (int index = 0; index < points.size(); ++index) {
            if (scoreIndex(index)) {
                break;
            }
        }
    }

    const int denom = std::max(1, out.evaluatedPoints);
    out.coverageApprox = static_cast<double>(out.matchedPoints) / static_cast<double>(denom);
    out.orientationApprox = out.evaluatedPoints > 0 ? orientationSum / static_cast<double>(out.evaluatedPoints) : 0.0;
    out.polarityApprox = out.evaluatedPoints > 0 ? polaritySum / static_cast<double>(out.evaluatedPoints) : 0.0;
    if (!out.rejectedByEarlyExit) {
        out.fastScore = totalWeight > 0.0f ? static_cast<double>(scoreSum / totalWeight) : 0.0;
    }
    if (out.fastScore <= 0.0 && out.rejectReason.empty()) {
        out.rejectReason = "zero_fast_score";
    }
    (void)distanceSum;
    return out;
}

CoarseCandidate FastPoseScorer::scorePoseFast(const TemplatePointSoA& points,
                                              const RotatedTemplateCache& rotationCache,
                                              const EdgeQueryContext& queryContext,
                                              const MatchPose& pose,
                                              int pyramidLevel,
                                              const FastScoreContext& context) const
{
    CoarseCandidate out;
    out.pose = pose;
    out.pyramidLevel = pyramidLevel;
    out.totalPointCount = points.size();
    const EdgeImageData* imageLevel = queryContext.fullEdgeData;

    if (points.size() < m_config.minTemplatePointsForScore || !imageLevel || imageLevel->empty()) {
        out.rejectReason = "insufficient_input";
        return out;
    }

    const RotatedTemplateView* rotated = m_config.enableRotatedTemplateCache
        ? rotationCache.findNearest(static_cast<float>(pose.theta))
        : nullptr;
    const float c = rotated ? rotated->cosTheta : std::cos(static_cast<float>(pose.theta));
    const float s = rotated ? rotated->sinTheta : std::sin(static_cast<float>(pose.theta));
    const float totalWeight = points.totalWeight > 0.0f ? points.totalWeight : static_cast<float>(std::max(1, points.size()));
    float scoreSum = 0.0f;
    float remainingWeight = totalWeight;
    double orientationSum = 0.0;
    double polaritySum = 0.0;
    const double searchRadius = std::max(1.0, static_cast<double>(m_config.maxDistanceForScore));
    const std::vector<int>* order = points.orderedIndices.empty() ? nullptr : &points.orderedIndices;

    const auto scoreIndex = [&](int index) {
        const size_t i = static_cast<size_t>(index);
        const float weight = templatePointWeight(points.weight[i], points.gradMag[i]);
        remainingWeight = std::max(0.0f, remainingWeight - weight);
        ++out.evaluatedPoints;

        const float rx = rotated ? rotated->rx[i] : (points.x[i] * c - points.y[i] * s);
        const float ry = rotated ? rotated->ry[i] : (points.x[i] * s + points.y[i] * c);
        const cv::Point2d predicted(static_cast<double>(pose.x + pose.scale * rx),
                                    static_cast<double>(pose.y + pose.scale * ry));
        float pointScore = 0.0f;
        cv::Point2d matched;
        cv::Point2d imageNormal;
        double gradientMag = 0.0;
        double distance = 0.0;
        if (queryNearestEdge(queryContext,
                             pyramidLevel,
                             predicted,
                             searchRadius,
                             matched,
                             imageNormal,
                             gradientMag,
                             distance,
                             nullptr)) {
            ++out.matchedPoints;
            const float dScore = static_cast<float>(distanceScore(distance));
            float predX = rotated ? rotated->rgx[i] : (points.gx[i] * c - points.gy[i] * s);
            float predY = rotated ? rotated->rgy[i] : (points.gx[i] * s + points.gy[i] * c);
            const float predNorm = std::sqrt(predX * predX + predY * predY);
            if (predNorm > 1e-6f) {
                predX /= predNorm;
                predY /= predNorm;
            }
            imageNormal = normalized(imageNormal);
            const float dot = std::clamp(static_cast<float>(predX * imageNormal.x + predY * imageNormal.y), -1.0f, 1.0f);
            const float angleDiffDeg = static_cast<float>(radToDeg(std::acos(dot)));
            const float oScore = m_config.useGradientOrientation ? static_cast<float>(orientationScore(angleDiffDeg)) : 1.0f;
            bool polarityOk = true;
            const EdgePolarity polarity = static_cast<EdgePolarity>(points.polarity[i]);
            if (polarity == EdgePolarity::DarkToBright) {
                polarityOk = dot >= 0.0f;
            } else if (polarity == EdgePolarity::BrightToDark) {
                polarityOk = dot <= 0.0f;
            }
            const float pScore = m_config.usePolarity ? static_cast<float>(polarityScore(polarityOk, polarity)) : 1.0f;
            orientationSum += oScore;
            polaritySum += pScore;
            pointScore = m_config.fastWDistance * dScore
                + m_config.fastWOrientation * oScore
                + m_config.fastWPolarity * pScore;
        }
        scoreSum += pointScore * weight;
        return shouldPrune(m_config, context, out.evaluatedPoints, scoreSum, remainingWeight, totalWeight, out);
    };

    if (order) {
        for (int index : *order) {
            if (scoreIndex(index)) {
                break;
            }
        }
    } else {
        for (int index = 0; index < points.size(); ++index) {
            if (scoreIndex(index)) {
                break;
            }
        }
    }

    const int denom = std::max(1, out.evaluatedPoints);
    out.coverageApprox = static_cast<double>(out.matchedPoints) / static_cast<double>(denom);
    out.orientationApprox = out.evaluatedPoints > 0 ? orientationSum / static_cast<double>(out.evaluatedPoints) : 0.0;
    out.polarityApprox = out.evaluatedPoints > 0 ? polaritySum / static_cast<double>(out.evaluatedPoints) : 0.0;
    if (!out.rejectedByEarlyExit) {
        out.fastScore = totalWeight > 0.0f ? static_cast<double>(scoreSum / totalWeight) : 0.0;
    }
    if (out.fastScore <= 0.0 && out.rejectReason.empty()) {
        out.rejectReason = "zero_fast_score";
    }
    return out;
}

CoarseCandidate FastPoseScorer::scorePoseWithContext(const ShapeTemplateModel& templateLevel,
                                                     const EdgeQueryContext& queryContext,
                                                     const MatchPose& pose,
                                                     int pyramidLevel,
                                                     const FastScoreContext& context) const
{
    TemplatePointSoA points = TemplatePointSoABuilder().build(templateLevel, m_config.enablePointOrdering);
    RotatedTemplateCache cache;
    cache.build(points, buildUniformThetaBinsRad(m_config.minAngleDeg,
                                                 m_config.maxAngleDeg,
                                                 std::max(0.25, m_config.rotatedCacheAngleStepDeg)));
    return scorePoseFast(points, cache, queryContext, pose, pyramidLevel, context);
}

CoarseCandidate FastPoseScorer::scorePoseDistanceField(const ShapeTemplateModel& templateLevel,
                                                       const EdgeImageData& imageLevel,
                                                       const MatchPose& pose,
                                                       int pyramidLevel,
                                                       const FastScoreContext& context) const
{
    CoarseCandidate out;
    out.pose = pose;
    out.pyramidLevel = pyramidLevel;
    out.totalPointCount = static_cast<int>(templateLevel.points.size());

    if (out.totalPointCount < m_config.minTemplatePointsForScore || imageLevel.empty()) {
        out.rejectReason = "insufficient_input";
        return out;
    }

    const std::vector<int> order = orderedPointIndices(templateLevel);
    double totalWeight = 0.0;
    for (const TemplatePoint& pt : templateLevel.points) {
        totalWeight += pointWeight(pt);
    }
    if (totalWeight <= 0.0) {
        totalWeight = static_cast<double>(std::max(1, out.totalPointCount));
    }

    double scoreSum = 0.0;
    double remainingWeight = totalWeight;
    double distanceSum = 0.0;
    double orientationSum = 0.0;
    double polaritySum = 0.0;

    const double c = std::cos(pose.theta);
    const double s = std::sin(pose.theta);
    const float maxDistance = std::max(0.1f, m_config.maxDistanceForScore);

    for (int index : order) {
        const TemplatePoint& pt = templateLevel.points[static_cast<size_t>(index)];
        const double weight = pointWeight(pt);
        remainingWeight = std::max(0.0, remainingWeight - weight);
        ++out.evaluatedPoints;

        const double sx = pt.position.x * pose.scale;
        const double sy = pt.position.y * pose.scale;
        const double px = pose.x + sx * c - sy * s;
        const double py = pose.y + sx * s + sy * c;
        double pointScore = 0.0;
        if (px >= 0.0 && py >= 0.0 && px < imageLevel.imageSize.width && py < imageLevel.imageSize.height) {
            const int ix = std::clamp(static_cast<int>(std::round(px)), 0, imageLevel.imageSize.width - 1);
            const int iy = std::clamp(static_cast<int>(std::round(py)), 0, imageLevel.imageSize.height - 1);
            const float d = sampleFloat(imageLevel.distanceMap, ix, iy, maxDistance + 1.0f);
            const double dScore = d <= maxDistance ? distanceScore(d) : 0.0;

            cv::Point2d imageNormal(1.0, 0.0);
            const int nx = sampleInt(imageLevel.nearestEdgeX, ix, iy, ix);
            const int ny = sampleInt(imageLevel.nearestEdgeY, ix, iy, iy);
            double imageGradMag = sampleFloat(imageLevel.gradMagMap, ix, iy, 0.0f);
            if (m_config.enableOrientationField && !imageLevel.orientationMap.empty()) {
                const float angle = sampleFloat(imageLevel.orientationMap, ix, iy, 0.0f);
                imageNormal = cv::Point2d(std::cos(angle), std::sin(angle));
            } else {
                imageNormal = normalized(cv::Point2d(sampleFloat(imageLevel.gradX, nx, ny),
                                                    sampleFloat(imageLevel.gradY, nx, ny)));
                imageGradMag = sampleFloat(imageLevel.gradMag, nx, ny, imageGradMag);
            }

            const cv::Point2d templateNormal = norm(pt.gradientDir) > 1e-6 ? pt.gradientDir : pt.normal;
            const cv::Point2d predNormal = normalized(cv::Point2d(templateNormal.x * c - templateNormal.y * s,
                                                                  templateNormal.x * s + templateNormal.y * c));
            const double oScore = m_config.useGradientOrientation
                ? orientationScore(radToDeg(angleBetweenUnitVectors(predNormal, normalized(imageNormal))))
                : 1.0;
            const double dot = predNormal.x * imageNormal.x + predNormal.y * imageNormal.y;
            bool polarityOk = true;
            if (pt.polarity == EdgePolarity::DarkToBright) {
                polarityOk = dot >= 0.0;
            } else if (pt.polarity == EdgePolarity::BrightToDark) {
                polarityOk = dot <= 0.0;
            }
            const double pScore = m_config.usePolarity ? polarityScore(polarityOk, pt.polarity) : 1.0;

            if (d <= maxDistance) {
                ++out.matchedPoints;
            }
            (void)imageGradMag;
            distanceSum += dScore;
            orientationSum += oScore;
            polaritySum += pScore;
            pointScore = static_cast<double>(m_config.fastWDistance) * dScore
                + static_cast<double>(m_config.fastWOrientation) * oScore
                + static_cast<double>(m_config.fastWPolarity) * pScore;
        }

        scoreSum += pointScore * weight;
        if (shouldPrune(m_config, context, out.evaluatedPoints, scoreSum, remainingWeight, totalWeight, out)) {
            break;
        }
    }

    const int denom = std::max(1, out.evaluatedPoints);
    out.coverageApprox = static_cast<double>(out.matchedPoints) / static_cast<double>(denom);
    out.orientationApprox = out.evaluatedPoints > 0 ? orientationSum / static_cast<double>(out.evaluatedPoints) : 0.0;
    out.polarityApprox = out.evaluatedPoints > 0 ? polaritySum / static_cast<double>(out.evaluatedPoints) : 0.0;
    const double distanceApprox = out.evaluatedPoints > 0 ? distanceSum / static_cast<double>(out.evaluatedPoints) : 0.0;
    if (!out.rejectedByEarlyExit) {
        out.fastScore = totalWeight > 0.0 ? scoreSum / totalWeight : 0.0;
    }
    if (out.fastScore <= 0.0 && out.rejectReason.empty()) {
        out.rejectReason = "zero_fast_score";
    }
    (void)distanceApprox;
    return out;
}

CoarseCandidate FastPoseScorer::scorePoseLocalSearch(const ShapeTemplateModel& templateLevel,
                                                     const EdgeImageData& imageLevel,
                                                     const MatchPose& pose,
                                                     int pyramidLevel,
                                                     const FastScoreContext& context) const
{
    CoarseCandidate out;
    out.pose = pose;
    out.pyramidLevel = pyramidLevel;
    out.totalPointCount = static_cast<int>(templateLevel.points.size());

    if (out.totalPointCount < m_config.minTemplatePointsForScore || imageLevel.empty()) {
        out.rejectReason = "insufficient_input";
        return out;
    }

    const std::vector<int> order = orderedPointIndices(templateLevel);
    double totalWeight = 0.0;
    for (const TemplatePoint& pt : templateLevel.points) {
        totalWeight += pointWeight(pt);
    }
    if (totalWeight <= 0.0) {
        totalWeight = static_cast<double>(std::max(1, out.totalPointCount));
    }

    double scoreSum = 0.0;
    double remainingWeight = totalWeight;
    double distanceSum = 0.0;
    double orientationSum = 0.0;
    double polaritySum = 0.0;
    const double searchRadius = std::max(1.0, m_config.fastDistanceSigma * 2.5);
    const double c = std::cos(pose.theta);
    const double s = std::sin(pose.theta);

    for (int index : order) {
        const TemplatePoint& pt = templateLevel.points[static_cast<size_t>(index)];
        const double weight = pointWeight(pt);
        remainingWeight = std::max(0.0, remainingWeight - weight);
        ++out.evaluatedPoints;
        const double sx = pt.position.x * pose.scale;
        const double sy = pt.position.y * pose.scale;
        const double px = pose.x + sx * c - sy * s;
        const double py = pose.y + sx * s + sy * c;
        double pointScore = 0.0;

        cv::Point2d imageNormal(1.0, 0.0);
        double gradientMag = 0.0;
        double distance = 0.0;
        if (fastNearestEdge(imageLevel, px, py, searchRadius, imageNormal, gradientMag, distance)) {
            ++out.matchedPoints;
            const cv::Point2d templateNormal = norm(pt.gradientDir) > 1e-6 ? pt.gradientDir : pt.normal;
            const cv::Point2d predNormal = normalized(cv::Point2d(templateNormal.x * c - templateNormal.y * s,
                                                                  templateNormal.x * s + templateNormal.y * c));
            const double dScore = distanceScore(distance);
            const double oScore = m_config.useGradientOrientation
                ? orientationScore(radToDeg(angleBetweenUnitVectors(predNormal, normalized(imageNormal))))
                : 1.0;
            const double dot = predNormal.x * imageNormal.x + predNormal.y * imageNormal.y;
            bool polarityOk = true;
            if (pt.polarity == EdgePolarity::DarkToBright) {
                polarityOk = dot >= 0.0;
            } else if (pt.polarity == EdgePolarity::BrightToDark) {
                polarityOk = dot <= 0.0;
            }
            const double pScore = m_config.usePolarity ? polarityScore(polarityOk, pt.polarity) : 1.0;
            (void)gradientMag;
            distanceSum += dScore;
            orientationSum += oScore;
            polaritySum += pScore;
            pointScore = static_cast<double>(m_config.fastWDistance) * dScore
                + static_cast<double>(m_config.fastWOrientation) * oScore
                + static_cast<double>(m_config.fastWPolarity) * pScore;
        }

        scoreSum += pointScore * weight;
        if (shouldPrune(m_config, context, out.evaluatedPoints, scoreSum, remainingWeight, totalWeight, out)) {
            break;
        }
    }

    const int denom = std::max(1, out.evaluatedPoints);
    out.coverageApprox = static_cast<double>(out.matchedPoints) / static_cast<double>(denom);
    out.orientationApprox = out.matchedPoints > 0 ? orientationSum / static_cast<double>(out.matchedPoints) : 0.0;
    out.polarityApprox = out.matchedPoints > 0 ? polaritySum / static_cast<double>(out.matchedPoints) : 0.0;
    if (!out.rejectedByEarlyExit) {
        out.fastScore = totalWeight > 0.0 ? scoreSum / totalWeight : 0.0;
    }
    if (out.fastScore <= 0.0 && out.rejectReason.empty()) {
        out.rejectReason = "zero_fast_score";
    }
    (void)distanceSum;
    return out;
}

std::vector<int> FastPoseScorer::orderedPointIndices(const ShapeTemplateModel& templateLevel) const
{
    std::vector<int> indices(static_cast<size_t>(templateLevel.points.size()));
    std::iota(indices.begin(), indices.end(), 0);
    if (!m_config.enablePointOrdering) {
        return indices;
    }
    std::stable_sort(indices.begin(), indices.end(), [&templateLevel](int lhs, int rhs) {
        const TemplatePoint& a = templateLevel.points[static_cast<size_t>(lhs)];
        const TemplatePoint& b = templateLevel.points[static_cast<size_t>(rhs)];
        const double sa = pointWeight(a);
        const double sb = pointWeight(b);
        if (std::abs(sa - sb) > 1e-12) {
            return sa > sb;
        }
        return a.id < b.id;
    });
    return indices;
}

double FastPoseScorer::distanceScore(double distance) const
{
    const double sigma = std::max(1e-6, static_cast<double>(m_config.distanceScoreSigma > 0.0f
        ? m_config.distanceScoreSigma
        : static_cast<float>(m_config.fastDistanceSigma)));
    return std::exp(-(distance * distance) / (2.0 * sigma * sigma));
}

double FastPoseScorer::orientationScore(double angleDiffDeg) const
{
    const double sigma = std::max(1e-6, static_cast<double>(m_config.orientationScoreSigmaDeg > 0.0f
        ? m_config.orientationScoreSigmaDeg
        : static_cast<float>(m_config.fastAngleSigmaDeg)));
    return std::exp(-(angleDiffDeg * angleDiffDeg) / (2.0 * sigma * sigma));
}

double FastPoseScorer::polarityScore(bool polarityOk, EdgePolarity polarity) const
{
    if (polarity == EdgePolarity::Any) {
        return 1.0;
    }
    return polarityOk ? 1.0 : 0.5;
}

} // namespace ShapeMatch
