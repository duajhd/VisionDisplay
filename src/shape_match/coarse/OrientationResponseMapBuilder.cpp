#include "shape_match/coarse/OrientationResponseMapBuilder.h"

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

int orientationBinFromVector(double x, double y, int binCount)
{
    const double angle = std::atan2(y, x);
    const double normalized = (angle + kPi) / (2.0 * kPi);
    int bin = static_cast<int>(std::floor(normalized * static_cast<double>(binCount)));
    return wrapBin(bin, binCount);
}

bool gradientAt(const EdgeImageData& edgeData, int x, int y, cv::Point2d& g, double& mag)
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
    if (!edgeData.gradMag.empty()) {
        mag = edgeData.gradMag.at<float>(y, x);
    } else if (!edgeData.gradMagMap.empty()) {
        mag = edgeData.gradMagMap.at<float>(y, x);
    } else {
        mag = 1.0;
    }
    return true;
}

} // namespace

OrientationResponseMap OrientationResponseMapBuilder::build(const EdgeImageData& edgeData,
                                                            int level,
                                                            const OrientationResponseConfig& config) const
{
    const auto start = std::chrono::steady_clock::now();
    OrientationResponseMap response;
    response.level = level;
    response.imageSize = edgeData.imageSize;
    response.orientationBinCount = std::max(1, config.orientationBinCount);
    response.perBinCounts.assign(static_cast<size_t>(response.orientationBinCount), 0);

    if (edgeData.imageSize.empty() || (edgeData.edgeMap.empty() && edgeData.edgePoints.empty())) {
        response.buildTimeMs = elapsedMs(start);
        return response;
    }

    response.binMaps.reserve(static_cast<size_t>(response.orientationBinCount));
    for (int i = 0; i < response.orientationBinCount; ++i) {
        response.binMaps.push_back(cv::Mat::zeros(edgeData.imageSize, CV_32F));
    }

    auto writePoint = [&](int x, int y) {
        cv::Point2d g;
        double mag = 1.0;
        if (!gradientAt(edgeData, x, y, g, mag)) {
            return;
        }
        const int bin = orientationBinFromVector(g.x, g.y, response.orientationBinCount);
        const float weight = static_cast<float>(config.useGradientMagnitudeWeight ? std::min(1.0, mag / 255.0) : 1.0);
        const int spread = config.enableOrientationSpreading ? std::max(0, config.orientationSpreadRadiusBins) : 0;
        for (int d = -spread; d <= spread; ++d) {
            const int b = wrapBin(bin + d, response.orientationBinCount);
            float& dst = response.binMaps[static_cast<size_t>(b)].at<float>(y, x);
            dst = std::max(dst, weight);
        }
        ++response.perBinCounts[static_cast<size_t>(bin)];
        ++response.usedEdgeCount;
    };

    if (!edgeData.edgePoints.empty()) {
        response.rawEdgeCount = static_cast<int>(edgeData.edgePoints.size());
        for (const cv::Point2d& p : edgeData.edgePoints) {
            const int x = static_cast<int>(std::round(p.x));
            const int y = static_cast<int>(std::round(p.y));
            if (x >= 0 && y >= 0 && x < edgeData.imageSize.width && y < edgeData.imageSize.height) {
                writePoint(x, y);
            }
        }
    } else {
        response.rawEdgeCount = cv::countNonZero(edgeData.edgeMap);
        for (int y = 0; y < edgeData.edgeMap.rows; ++y) {
            const uchar* edgeRow = edgeData.edgeMap.ptr<uchar>(y);
            for (int x = 0; x < edgeData.edgeMap.cols; ++x) {
                if (edgeRow[x] != 0) {
                    writePoint(x, y);
                }
            }
        }
    }

    if (config.enableSpatialSpreading && config.spatialSpreadRadiusPx > 0) {
        const int radius = std::max(1, config.spatialSpreadRadiusPx);
        const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(radius * 2 + 1, radius * 2 + 1));
        for (cv::Mat& map : response.binMaps) {
            cv::dilate(map, map, kernel);
        }
    }

    response.valid = response.usedEdgeCount > 0;
    response.buildTimeMs = elapsedMs(start);
    return response;
}

} // namespace ShapeMatch
