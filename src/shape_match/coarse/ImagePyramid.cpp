#include "shape_match/coarse/ImagePyramid.h"

#include "shape_match/coarse/DistanceFieldBuilder.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <numeric>

namespace ShapeMatch {

namespace {

void rebuildEdgeVectors(EdgeImageData& data)
{
    data.edgePoints.clear();
    data.edgeNormals.clear();
    if (data.edgeMap.empty()) {
        return;
    }

    for (int y = 0; y < data.edgeMap.rows; ++y) {
        for (int x = 0; x < data.edgeMap.cols; ++x) {
            if (data.edgeMap.at<uchar>(y, x) == 0) {
                continue;
            }
            data.edgePoints.emplace_back(static_cast<double>(x), static_cast<double>(y));
            cv::Point2d g(1.0, 0.0);
            if (!data.gradX.empty() && !data.gradY.empty()) {
                g.x = data.gradX.at<float>(y, x);
                g.y = data.gradY.at<float>(y, x);
            }
            data.edgeNormals.push_back(normalized(g));
        }
    }
}

double elapsedMsSince(std::chrono::steady_clock::time_point t0)
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

cv::Mat scaledOrEmpty(const cv::Mat& src, const cv::Size& size, int interpolation)
{
    if (src.empty()) {
        return {};
    }
    cv::Mat dst;
    cv::resize(src, dst, size, 0.0, 0.0, interpolation);
    return dst;
}

} // namespace

bool ImagePyramid::build(const EdgeImageData& baseEdgeData, int levels, const CoarseMatchConfig& config)
{
    m_levels.clear();
    m_scales.clear();
    m_distanceFieldBuildTimesMs.clear();
    m_skippedDistanceField.clear();
    if (baseEdgeData.empty() || levels <= 0) {
        return false;
    }

    levels = std::max(1, levels);
    m_levels.push_back(baseEdgeData);
    DistanceFieldBuilder distanceFieldBuilder;
    auto dfT0 = std::chrono::steady_clock::now();
    const long long basePixels = static_cast<long long>(baseEdgeData.imageSize.width) * static_cast<long long>(baseEdgeData.imageSize.height);
    const bool largeImageRoiMode = config.roiDistanceField.enableRoiDistanceField
        && basePixels >= config.roiDistanceField.largeImageMinPixelsForRoiDistanceField;
    const bool buildLevel0Full = !largeImageRoiMode || config.roiDistanceField.buildFullDistanceFieldForLevel0;
    if (buildLevel0Full) {
        distanceFieldBuilder.build(m_levels.back(), config);
        m_skippedDistanceField.push_back(false);
    } else {
        m_levels.back().hasDistanceField = false;
        m_levels.back().distanceMap.release();
        m_levels.back().nearestEdgeX.release();
        m_levels.back().nearestEdgeY.release();
        m_levels.back().orientationMap.release();
        m_levels.back().gradMagMap.release();
        m_skippedDistanceField.push_back(true);
    }
    m_distanceFieldBuildTimesMs.push_back(buildLevel0Full ? elapsedMsSince(dfT0) : 0.0);
    m_scales.push_back(1.0);

    for (int levelIndex = 1; levelIndex < levels; ++levelIndex) {
        const EdgeImageData& prev = m_levels.back();
        const int nextWidth = std::max(1, prev.imageSize.width / 2);
        const int nextHeight = std::max(1, prev.imageSize.height / 2);
        if (nextWidth == prev.imageSize.width && nextHeight == prev.imageSize.height) {
            break;
        }

        EdgeImageData next;
        next.imageSize = cv::Size(nextWidth, nextHeight);
        cv::Mat edgeFloat;
        if (!prev.edgeMap.empty()) {
            prev.edgeMap.convertTo(edgeFloat, CV_32F, 1.0 / 255.0);
            cv::resize(edgeFloat, edgeFloat, next.imageSize, 0.0, 0.0, cv::INTER_AREA);
            cv::threshold(edgeFloat, next.edgeMap, 0.05, 255.0, cv::THRESH_BINARY);
            next.edgeMap.convertTo(next.edgeMap, CV_8U);
        }
        next.gradX = scaledOrEmpty(prev.gradX, next.imageSize, cv::INTER_AREA);
        next.gradY = scaledOrEmpty(prev.gradY, next.imageSize, cv::INTER_AREA);
        next.gradMag = scaledOrEmpty(prev.gradMag, next.imageSize, cv::INTER_AREA);
        rebuildEdgeVectors(next);
        dfT0 = std::chrono::steady_clock::now();
        const bool buildFull = !largeImageRoiMode
            || (levelIndex == 1 ? config.roiDistanceField.buildFullDistanceFieldForLevel1
                                : config.roiDistanceField.buildFullDistanceFieldForCoarseLevels);
        if (buildFull) {
            distanceFieldBuilder.build(next, config);
            m_skippedDistanceField.push_back(false);
        } else {
            next.hasDistanceField = false;
            m_skippedDistanceField.push_back(true);
        }
        m_distanceFieldBuildTimesMs.push_back(buildFull ? elapsedMsSince(dfT0) : 0.0);
        m_levels.push_back(std::move(next));
        m_scales.push_back(m_scales.back() * 0.5);
    }
    return !m_levels.empty();
}

int ImagePyramid::levelCount() const
{
    return static_cast<int>(m_levels.size());
}

const EdgeImageData& ImagePyramid::level(int levelIndex) const
{
    return m_levels.at(static_cast<size_t>(levelIndex));
}

double ImagePyramid::scaleOfLevel(int levelIndex) const
{
    return m_scales.at(static_cast<size_t>(levelIndex));
}

double ImagePyramid::distanceFieldBuildTimeMs(int levelIndex) const
{
    return m_distanceFieldBuildTimesMs.at(static_cast<size_t>(levelIndex));
}

double ImagePyramid::totalDistanceFieldBuildTimeMs() const
{
    return std::accumulate(m_distanceFieldBuildTimesMs.begin(), m_distanceFieldBuildTimesMs.end(), 0.0);
}

bool ImagePyramid::skippedFullDistanceField(int levelIndex) const
{
    return m_skippedDistanceField.at(static_cast<size_t>(levelIndex));
}

} // namespace ShapeMatch
