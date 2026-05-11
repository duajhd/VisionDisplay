#pragma once

#include "VisionTools/CaliperTypes.h"
#include "VisionTools/EdgePoint.h"
#include "VisionTools/VisionTools_global.h"

#include <QString>

#include <vector>

namespace VisionTools {

enum class CircleSearchDirection
{
    Inward,
    Outward
};

struct VISIONTOOLS_API CircleSearchRegion
{
    double centerX = 0.0;
    double centerY = 0.0;
    double radius = 0.0;
    double searchLength = 0.0;
    double caliperWidth = 0.0;
    double startAngleDeg = 0.0;
    double spanAngleDeg = 360.0;
    int caliperCount = 0;
    CircleSearchDirection searchDirection = CircleSearchDirection::Outward;
};

struct VISIONTOOLS_API CircleModel
{
    double centerX = 0.0;
    double centerY = 0.0;
    double radius = 0.0;
};

struct VISIONTOOLS_API CircleFitParams
{
    double maxResidual = 2.0;
    int minInlierCount = 3;
    int maxIterations = 20;
    double damping = 1.0e-3;
    bool enableRansac = true;
    int ransacIterations = 160;
    double ransacResidual = 2.0;
    bool enableHuber = true;
    double huberDelta = 1.5;
};

struct VISIONTOOLS_API CircleFitResult
{
    bool ok = false;
    CircleModel circle;
    std::vector<EdgePoint> inputPoints;
    std::vector<EdgePoint> inlierPoints;
    std::vector<EdgePoint> outlierPoints;
    double rmsError = 0.0;
    double maxError = 0.0;
    double score = 0.0;
    QString message;
};

struct VISIONTOOLS_API FindCircleParams
{
    CaliperParams caliperParams;
    CircleFitParams fitParams;
    double minEdgeResponse = 0.0;
};

struct VISIONTOOLS_API FindCircleResult
{
    bool ok = false;
    CircleSearchRegion searchRegion;
    std::vector<CaliperRegion> calipers;
    std::vector<CaliperResult> caliperResults;
    std::vector<EdgePoint> edgePoints;
    std::vector<EdgePoint> inputPoints;
    std::vector<EdgePoint> inlierPoints;
    std::vector<EdgePoint> outlierPoints;
    double rmsError = 0.0;
    double maxError = 0.0;
    double score = 0.0;
    CircleFitResult fitResult;
    QString message;
};

} // namespace VisionTools
