#include "shape_match/coarse/DistanceFieldBuilder.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace ShapeMatch {

namespace {

float sampleFloat(const cv::Mat& mat, int x, int y, float fallback = 0.0f)
{
    if (mat.empty() || x < 0 || y < 0 || x >= mat.cols || y >= mat.rows) {
        return fallback;
    }
    return mat.at<float>(y, x);
}

cv::Point2d normalAt(const EdgeImageData& data, int x, int y)
{
    const cv::Point2d g(sampleFloat(data.gradX, x, y), sampleFloat(data.gradY, x, y));
    return normalized(g);
}

} // namespace

bool DistanceFieldBuilder::build(EdgeImageData& edgeData, const CoarseMatchConfig& config) const
{
    edgeData.hasDistanceField = false;
    edgeData.distanceMap.release();
    edgeData.nearestEdgeX.release();
    edgeData.nearestEdgeY.release();
    edgeData.orientationMap.release();
    edgeData.gradMagMap.release();

    if (!config.enableDistanceFieldScoring || edgeData.edgeMap.empty()) {
        return false;
    }

    cv::Mat binary;
    cv::threshold(edgeData.edgeMap, binary, 0, 255, cv::THRESH_BINARY);
    if (cv::countNonZero(binary) <= 0) {
        return false;
    }

    cv::Mat inverse;
    cv::bitwise_not(binary, inverse);
    cv::Mat labels;
    cv::distanceTransform(inverse,
                          edgeData.distanceMap,
                          labels,
                          cv::DIST_L2,
                          cv::DIST_MASK_3,
                          cv::DIST_LABEL_PIXEL);
    if (edgeData.distanceMap.empty() || edgeData.distanceMap.type() != CV_32F) {
        return false;
    }

    edgeData.nearestEdgeX.create(edgeData.edgeMap.size(), CV_32S);
    edgeData.nearestEdgeY.create(edgeData.edgeMap.size(), CV_32S);
    edgeData.orientationMap.create(edgeData.edgeMap.size(), CV_32F);
    edgeData.gradMagMap.create(edgeData.edgeMap.size(), CV_32F);
    edgeData.nearestEdgeX.setTo(-1);
    edgeData.nearestEdgeY.setTo(-1);
    edgeData.orientationMap.setTo(0.0f);
    edgeData.gradMagMap.setTo(0.0f);

    std::unordered_map<int, cv::Point> labelToEdge;
    labelToEdge.reserve(static_cast<size_t>(std::max(1, cv::countNonZero(binary))));
    for (int y = 0; y < binary.rows; ++y) {
        const uchar* edgeRow = binary.ptr<uchar>(y);
        const int* labelRow = labels.ptr<int>(y);
        for (int x = 0; x < binary.cols; ++x) {
            if (edgeRow[x] == 0) {
                continue;
            }
            const int label = labelRow[x];
            if (label > 0 && labelToEdge.find(label) == labelToEdge.end()) {
                labelToEdge.emplace(label, cv::Point(x, y));
            }
        }
    }

    for (int y = 0; y < labels.rows; ++y) {
        const int* labelRow = labels.ptr<int>(y);
        int* nearestX = edgeData.nearestEdgeX.ptr<int>(y);
        int* nearestY = edgeData.nearestEdgeY.ptr<int>(y);
        float* orientation = edgeData.orientationMap.ptr<float>(y);
        float* mag = edgeData.gradMagMap.ptr<float>(y);
        for (int x = 0; x < labels.cols; ++x) {
            cv::Point nearest(x, y);
            const auto found = labelToEdge.find(labelRow[x]);
            if (found != labelToEdge.end()) {
                nearest = found->second;
            } else if (binary.at<uchar>(y, x) == 0) {
                nearest = cv::Point(x, y);
            }

            nearestX[x] = nearest.x;
            nearestY[x] = nearest.y;
            const cv::Point2d n = normalAt(edgeData, nearest.x, nearest.y);
            orientation[x] = static_cast<float>(std::atan2(n.y, n.x));
            mag[x] = sampleFloat(edgeData.gradMag, nearest.x, nearest.y, binary.at<uchar>(nearest.y, nearest.x) ? 255.0f : 0.0f);
        }
    }

    edgeData.hasDistanceField = true;
    return true;
}

} // namespace ShapeMatch
