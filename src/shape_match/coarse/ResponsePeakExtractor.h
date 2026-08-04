#pragma once

#include "shape_match/coarse/OrientationResponseConfig.h"
#include "shape_match/core/ShapeMatchTypes.h"

#include <opencv2/core.hpp>

#include <vector>

namespace ShapeMatch {

struct ResponsePeak
{
    MatchPose pose;

    double responseScore = 0.0;

    int thetaBin = -1;
    int x = 0;
    int y = 0;

    int tileRow = -1;
    int tileCol = -1;
};

class ResponsePeakExtractor
{
public:
    std::vector<ResponsePeak> extractPeaks(const cv::Mat& responseXY,
                                           int thetaBin,
                                           double thetaRad,
                                           const OrientationResponseConfig& config) const;

    std::vector<ResponsePeak> mergeAndNms(const std::vector<ResponsePeak>& peaks,
                                          const OrientationResponseConfig& config) const;
};

} // namespace ShapeMatch
