#include "shape_match/pipeline_v2/DirectionalDistanceFieldBuilder.h"

#include "shape_match/core/ShapeMatchTypes.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace ShapeMatch {

namespace {

double elapsedMs(std::chrono::steady_clock::time_point start)
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

int wrapBin(int bin, int count)
{
    if (count <= 0) {
        return 0;
    }
    bin %= count;
    return bin < 0 ? bin + count : bin;
}

int binFromVector(double x, double y, int count)
{
    const double angle = std::atan2(y, x);
    return wrapBin(static_cast<int>(std::floor((angle + kPi) / (2.0 * kPi) * count)), count);
}

bool gradientAt(const EdgeImageData& edgeData, int x, int y, cv::Point2d& g)
{
    double gx = 0.0;
    double gy = 0.0;
    if (!edgeData.gradX.empty() && !edgeData.gradY.empty()) {
        gx = edgeData.gradX.at<float>(y, x);
        gy = edgeData.gradY.at<float>(y, x);
    } else if (!edgeData.orientationMap.empty()) {
        const float a = edgeData.orientationMap.at<float>(y, x);
        gx = std::cos(a);
        gy = std::sin(a);
    }
    const double n = std::sqrt(gx * gx + gy * gy);
    if (n <= 1e-6) {
        return false;
    }
    g = cv::Point2d(gx / n, gy / n);
    return true;
}

} // namespace

DirectionalDistanceField DirectionalDistanceFieldBuilder::build(const EdgeImageData& edgeData,
                                                               int level,
                                                               const DirectionalChamferConfig& config) const
{
    const auto start = std::chrono::steady_clock::now();
    DirectionalDistanceField field;
    field.level = level;
    field.imageSize = edgeData.imageSize;
    field.orientationBinCount = std::max(1, config.orientationBinCount);
    field.perBinEdgeCounts.assign(static_cast<size_t>(field.orientationBinCount), 0);
    if (edgeData.imageSize.empty() || (edgeData.edgeMap.empty() && edgeData.edgePoints.empty())) {
        field.buildTimeMs = elapsedMs(start);
        return field;
    }

    std::vector<cv::Mat> masks;
    masks.reserve(static_cast<size_t>(field.orientationBinCount));
    for (int i = 0; i < field.orientationBinCount; ++i) {
        masks.push_back(cv::Mat(edgeData.imageSize, CV_8U, cv::Scalar(255)));
    }

    auto addPoint = [&](int x, int y) {
        if (x < 0 || y < 0 || x >= edgeData.imageSize.width || y >= edgeData.imageSize.height) {
            return;
        }
        cv::Point2d g;
        if (!gradientAt(edgeData, x, y, g)) {
            return;
        }
        const int bin = binFromVector(g.x, g.y, field.orientationBinCount);
        const int spread = std::max(0, config.orientationSpreadRadiusBins);
        for (int d = -spread; d <= spread; ++d) {
            masks[static_cast<size_t>(wrapBin(bin + d, field.orientationBinCount))].at<uchar>(y, x) = 0;
        }
        ++field.perBinEdgeCounts[static_cast<size_t>(bin)];
    };

    if (!edgeData.edgePoints.empty()) {
        for (const cv::Point2d& p : edgeData.edgePoints) {
            addPoint(static_cast<int>(std::round(p.x)), static_cast<int>(std::round(p.y)));
        }
    } else {
        for (int y = 0; y < edgeData.edgeMap.rows; ++y) {
            const uchar* row = edgeData.edgeMap.ptr<uchar>(y);
            for (int x = 0; x < edgeData.edgeMap.cols; ++x) {
                if (row[x] != 0) {
                    addPoint(x, y);
                }
            }
        }
    }

    field.distanceBins.reserve(static_cast<size_t>(field.orientationBinCount));
    bool anyEdges = false;
    for (int i = 0; i < field.orientationBinCount; ++i) {
        cv::Mat dist;
        if (field.perBinEdgeCounts[static_cast<size_t>(i)] > 0) {
            cv::distanceTransform(masks[static_cast<size_t>(i)], dist, cv::DIST_L2, 3);
            anyEdges = true;
        } else {
            dist = cv::Mat(edgeData.imageSize, CV_32F, cv::Scalar(static_cast<float>(config.maxDistancePx * 4.0)));
        }
        field.distanceBins.push_back(dist);
    }
    field.valid = anyEdges;
    field.buildTimeMs = elapsedMs(start);
    return field;
}

} // namespace ShapeMatch
