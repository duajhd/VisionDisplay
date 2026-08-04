#pragma once

#include "shape_match/coarse/ResponsePeakExtractor.h"

#include <opencv2/core.hpp>

#include <vector>

namespace ShapeMatch {

struct ResponseDebugReport
{
    int level = -1;
    int imageWidth = 0;
    int imageHeight = 0;

    int orientationBinCount = 0;
    int thetaBinCount = 0;

    int rawEdgeCount = 0;
    int usedEdgeCount = 0;

    int responseTemplatePointCount = 0;

    int totalPeaksBeforeNms = 0;
    int totalPeaksAfterNms = 0;

    double buildResponseMapMs = 0.0;
    double buildTemplateMs = 0.0;
    double evaluateResponseMs = 0.0;
    double peakExtractMs = 0.0;
    double totalMs = 0.0;

    std::vector<ResponsePeak> topPeaks;

    cv::Mat maxResponseXY;
    cv::Mat bestThetaXY;
    cv::Mat edgeOverlayBase;
};

} // namespace ShapeMatch
