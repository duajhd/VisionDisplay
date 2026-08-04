#pragma once

#include <opencv2/core.hpp>

#include <vector>

namespace ShapeMatch {

struct OrientationResponseMap
{
    int level = -1;
    cv::Size imageSize;

    int orientationBinCount = 0;
    std::vector<cv::Mat> binMaps;

    bool valid = false;

    int rawEdgeCount = 0;
    int usedEdgeCount = 0;
    std::vector<int> perBinCounts;

    double buildTimeMs = 0.0;

    bool empty() const;
};

} // namespace ShapeMatch
