#include "shape_match/core/EdgeImageData.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ShapeMatch {

namespace {

float sampleFloat(const cv::Mat& mat, int x, int y, float fallback = 0.0f)
{
    if (mat.empty() || x < 0 || y < 0 || x >= mat.cols || y >= mat.rows) {
        return fallback;
    }
    return mat.at<float>(y, x);
}

} // namespace

bool EdgeImageData::empty() const
{
    return imageSize.width <= 0 || imageSize.height <= 0 || (edgePoints.empty() && edgeMap.empty());
}

bool EdgeImageData::isInside(const cv::Point2d& p) const
{
    return p.x >= 0.0 && p.y >= 0.0 && p.x < imageSize.width && p.y < imageSize.height;
}

bool EdgeImageData::hasNearestEdgeField() const
{
    return hasDistanceField
        && !distanceMap.empty()
        && !nearestEdgeX.empty()
        && !nearestEdgeY.empty()
        && distanceMap.type() == CV_32F
        && nearestEdgeX.type() == CV_32S
        && nearestEdgeY.type() == CV_32S
        && distanceMap.size() == nearestEdgeX.size()
        && distanceMap.size() == nearestEdgeY.size();
}

bool EdgeImageData::findNearestEdgeFast(const cv::Point2d& predictedPt,
                                        double searchRadiusPx,
                                        cv::Point2d& matchedPt,
                                        cv::Point2d& imageNormal,
                                        double& gradientMagValue,
                                        double& distance) const
{
    if (!isInside(predictedPt) || !hasNearestEdgeField()) {
        return false;
    }

    const int x = std::clamp(static_cast<int>(std::round(predictedPt.x)), 0, distanceMap.cols - 1);
    const int y = std::clamp(static_cast<int>(std::round(predictedPt.y)), 0, distanceMap.rows - 1);
    const float d = distanceMap.at<float>(y, x);
    if (!std::isfinite(d) || d > searchRadiusPx) {
        return false;
    }

    const int nx = nearestEdgeX.at<int>(y, x);
    const int ny = nearestEdgeY.at<int>(y, x);
    if (nx < 0 || ny < 0 || nx >= imageSize.width || ny >= imageSize.height) {
        return false;
    }

    matchedPt = cv::Point2d(static_cast<double>(nx), static_cast<double>(ny));
    const cv::Point2d residual = matchedPt - predictedPt;
    distance = std::sqrt(residual.x * residual.x + residual.y * residual.y);
    if (distance > searchRadiusPx) {
        return false;
    }

    if (!orientationMap.empty() && orientationMap.type() == CV_32F && orientationMap.size() == distanceMap.size()) {
        const double a = static_cast<double>(orientationMap.at<float>(y, x));
        imageNormal = normalized(cv::Point2d(std::cos(a), std::sin(a)));
    } else {
        const cv::Point2d g(sampleFloat(gradX, nx, ny), sampleFloat(gradY, nx, ny));
        imageNormal = norm(g) > 1e-6 ? normalized(g) : cv::Point2d(1.0, 0.0);
    }

    if (!gradMagMap.empty() && gradMagMap.type() == CV_32F && gradMagMap.size() == distanceMap.size()) {
        gradientMagValue = static_cast<double>(gradMagMap.at<float>(y, x));
    } else {
        gradientMagValue = sampleFloat(gradMag, nx, ny, 0.0f);
    }
    return true;
}

bool EdgeImageData::findNearestEdgeLocalWindow(const cv::Point2d& predictedPt,
                                               double searchRadiusPx,
                                               cv::Point2d& matchedPt,
                                               cv::Point2d& imageNormal,
                                               double& gradientMagValue,
                                               double& distance) const
{
    if (!isInside(predictedPt) || edgeMap.empty()) {
        return false;
    }

    double bestD2 = std::numeric_limits<double>::max();
    const int r = std::max(1, static_cast<int>(std::ceil(searchRadiusPx)));
    const int cx = static_cast<int>(std::round(predictedPt.x));
    const int cy = static_cast<int>(std::round(predictedPt.y));
    for (int y = std::max(0, cy - r); y <= std::min(edgeMap.rows - 1, cy + r); ++y) {
        for (int x = std::max(0, cx - r); x <= std::min(edgeMap.cols - 1, cx + r); ++x) {
            if (edgeMap.at<uchar>(y, x) == 0) {
                continue;
            }
            const cv::Point2d e(static_cast<double>(x), static_cast<double>(y));
            const cv::Point2d d = e - predictedPt;
            const double d2 = d.x * d.x + d.y * d.y;
            if (d2 <= searchRadiusPx * searchRadiusPx && d2 < bestD2) {
                bestD2 = d2;
                matchedPt = e;
            }
        }
    }
    if (bestD2 == std::numeric_limits<double>::max()) {
        return false;
    }

    distance = std::sqrt(bestD2);
    const int mx = std::clamp(static_cast<int>(std::round(matchedPt.x)), 0, imageSize.width - 1);
    const int my = std::clamp(static_cast<int>(std::round(matchedPt.y)), 0, imageSize.height - 1);
    const cv::Point2d g(sampleFloat(gradX, mx, my), sampleFloat(gradY, mx, my));
    imageNormal = norm(g) > 1e-6 ? normalized(g) : cv::Point2d(1.0, 0.0);
    gradientMagValue = sampleFloat(gradMag, mx, my, 0.0f);
    return true;
}

bool EdgeImageData::findNearestEdgeLinearScanDebugOnly(const cv::Point2d& predictedPt,
                                                       double searchRadiusPx,
                                                       cv::Point2d& matchedPt,
                                                       cv::Point2d& imageNormal,
                                                       double& gradientMagValue,
                                                       double& distance) const
{
    if (!isInside(predictedPt) || edgePoints.empty()) {
        return false;
    }

    double bestD2 = std::numeric_limits<double>::max();
    int bestIndex = -1;
    const double r2 = searchRadiusPx * searchRadiusPx;
    for (size_t i = 0; i < edgePoints.size(); ++i) {
        const cv::Point2d d = edgePoints[i] - predictedPt;
        const double d2 = d.x * d.x + d.y * d.y;
        if (d2 <= r2 && d2 < bestD2) {
            bestD2 = d2;
            bestIndex = static_cast<int>(i);
        }
    }
    if (bestIndex < 0) {
        return false;
    }

    matchedPt = edgePoints[static_cast<size_t>(bestIndex)];
    distance = std::sqrt(bestD2);
    const int mx = std::clamp(static_cast<int>(std::round(matchedPt.x)), 0, imageSize.width - 1);
    const int my = std::clamp(static_cast<int>(std::round(matchedPt.y)), 0, imageSize.height - 1);
    const cv::Point2d g(sampleFloat(gradX, mx, my), sampleFloat(gradY, mx, my));
    gradientMagValue = sampleFloat(gradMag, mx, my, 0.0f);
    if (norm(g) > 1e-6) {
        imageNormal = normalized(g);
    } else if (static_cast<size_t>(bestIndex) < edgeNormals.size()) {
        imageNormal = normalized(edgeNormals[static_cast<size_t>(bestIndex)]);
    } else {
        imageNormal = cv::Point2d(1.0, 0.0);
    }
    return true;
}

bool EdgeImageData::findNearestEdge(const cv::Point2d& predictedPt,
                                    double searchRadiusPx,
                                    cv::Point2d& matchedPt,
                                    cv::Point2d& imageNormal,
                                    double& gradientMagValue,
                                    double& distance) const
{
    if (findNearestEdgeFast(predictedPt, searchRadiusPx, matchedPt, imageNormal, gradientMagValue, distance)) {
        return true;
    }
    if (findNearestEdgeLocalWindow(predictedPt, searchRadiusPx, matchedPt, imageNormal, gradientMagValue, distance)) {
        return true;
    }

    const long long pixelCount = static_cast<long long>(imageSize.width) * static_cast<long long>(imageSize.height);
    if (pixelCount <= 1000000LL || edgePoints.size() <= 5000) {
        return findNearestEdgeLinearScanDebugOnly(predictedPt,
                                                 searchRadiusPx,
                                                 matchedPt,
                                                 imageNormal,
                                                 gradientMagValue,
                                                 distance);
    }
    return false;
}

EdgeImageData EdgeImageData::createFromTemplateAndPose(const ShapeTemplateModel& model,
                                                       const MatchPose& pose,
                                                       cv::Size size)
{
    EdgeImageData data;
    data.imageSize = size;
    data.edgeMap = cv::Mat::zeros(size, CV_8U);
    data.gradX = cv::Mat::zeros(size, CV_32F);
    data.gradY = cv::Mat::zeros(size, CV_32F);
    data.gradMag = cv::Mat::zeros(size, CV_32F);
    data.edgePoints.reserve(model.points.size());
    data.edgeNormals.reserve(model.points.size());

    for (const TemplatePoint& tp : model.points) {
        const cv::Point2d p = pose.transformPoint(tp.position);
        if (!data.isInside(p)) {
            continue;
        }
        const cv::Point2d n = normalized(pose.rotateVector(norm(tp.normal) > 1e-6 ? tp.normal : tp.gradientDir));
        data.edgePoints.push_back(p);
        data.edgeNormals.push_back(n);
        const int x = std::clamp(static_cast<int>(std::round(p.x)), 0, size.width - 1);
        const int y = std::clamp(static_cast<int>(std::round(p.y)), 0, size.height - 1);
        data.edgeMap.at<uchar>(y, x) = 255;
        data.gradX.at<float>(y, x) = static_cast<float>(n.x);
        data.gradY.at<float>(y, x) = static_cast<float>(n.y);
        data.gradMag.at<float>(y, x) = static_cast<float>(tp.gradientMag);
    }

    return data;
}

} // namespace ShapeMatch
