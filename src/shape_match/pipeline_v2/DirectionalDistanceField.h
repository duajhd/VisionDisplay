#pragma once

#include <opencv2/core.hpp>

#include <vector>

namespace ShapeMatch {

struct DirectionalDistanceField
{
    int level = -1;
    cv::Size imageSize;
    int orientationBinCount = 0;

    std::vector<cv::Mat> distanceBins;
    std::vector<int> perBinEdgeCounts;

    bool valid = false;
    double buildTimeMs = 0.0;
};

} // namespace ShapeMatch
