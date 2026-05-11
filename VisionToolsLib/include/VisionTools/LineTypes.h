#pragma once

#include "VisionTools/CaliperTypes.h"
#include "VisionTools/EdgePoint.h"
#include "VisionTools/VisionTools_global.h"

#include <QString>

#include <vector>

namespace VisionTools {

struct VISIONTOOLS_API LineSearchRegion
{
    double centerX = 0.0;
    double centerY = 0.0;
    double length = 0.0;
    double searchLength = 0.0;
    double angleDeg = 0.0;
    int caliperCount = 0;
    double caliperWidth = 0.0;
};

struct VISIONTOOLS_API LineModel
{
    double theta = 0.0;
    double rho = 0.0;
    double x1 = 0.0;
    double y1 = 0.0;
    double x2 = 0.0;
    double y2 = 0.0;
    double nx = 0.0;
    double ny = 0.0;
};

struct VISIONTOOLS_API LineFitParams
{
    double maxResidual = 2.0;
    int minInlierCount = 2;
    bool enableRansac = true;
    int ransacIterations = 100;
    double ransacResidual = 2.0;
    bool enableHuber = true;
    double huberDelta = 1.5;
    int robustIterations = 3;
};

struct VISIONTOOLS_API LineFitResult
{
    bool ok = false;
    LineModel line;
    std::vector<EdgePoint> inputPoints;
    std::vector<EdgePoint> inlierPoints;
    std::vector<EdgePoint> outlierPoints;
    double rmsError = 0.0;
    double maxError = 0.0;
    double score = 0.0;
    QString message;
};

struct VISIONTOOLS_API FindLineParams
{
    CaliperParams caliperParams;
    LineFitParams fitParams;
    double minEdgeResponse = 0.0;
};

struct VISIONTOOLS_API FindLineResult
{
    bool ok = false;
    LineSearchRegion searchRegion;
    std::vector<CaliperRegion> calipers;
    std::vector<CaliperResult> caliperResults;
    std::vector<EdgePoint> edgePoints;
    std::vector<EdgePoint> inputPoints;
    std::vector<EdgePoint> inlierPoints;
    std::vector<EdgePoint> outlierPoints;
    double rmsError = 0.0;
    double maxError = 0.0;
    double score = 0.0;
    LineFitResult fitResult;
    QString message;
};

} // namespace VisionTools
