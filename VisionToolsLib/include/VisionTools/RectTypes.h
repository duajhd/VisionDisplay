#pragma once

#include "VisionTools/CaliperTypes.h"
#include "VisionTools/LineTypes.h"
#include "VisionTools/VisionTools_global.h"

#include <QString>

#include <vector>

namespace VisionTools {

enum class RectEdgeId
{
    Top,
    Bottom,
    Left,
    Right
};

enum class RectSearchDirection
{
    FromOutsideToInside,
    FromInsideToOutside
};

struct VISIONTOOLS_API Point2D
{
    double x = 0.0;
    double y = 0.0;
};

struct VISIONTOOLS_API RectModel
{
    double centerX = 0.0;
    double centerY = 0.0;
    double width = 0.0;
    double height = 0.0;
    double angleDeg = 0.0;

    Point2D topLeft;
    Point2D topRight;
    Point2D bottomRight;
    Point2D bottomLeft;
};

struct VISIONTOOLS_API FindRectParams
{
    double centerX = 0.0;
    double centerY = 0.0;
    double expectedWidth = 100.0;
    double expectedHeight = 80.0;
    double angleDeg = 0.0;

    int calipersPerEdge = 15;
    double searchLength = 40.0;
    double projectionWidth = 8.0;

    EdgePolarity polarity = EdgePolarity::Any;
    EdgeSelection selection = EdgeSelection::Strongest;

    double minResponse = 5.0;
    double maxLineResidual = 2.0;
    double maxParallelAngleErrorDeg = 2.0;
    double maxOrthogonalAngleErrorDeg = 2.0;
    double maxSizeErrorRatio = 0.2;

    RectSearchDirection searchDirection = RectSearchDirection::FromOutsideToInside;

    bool enforceParallel = true;
    bool enforceOrthogonal = true;
};

struct VISIONTOOLS_API FindRectEdgeResult
{
    RectEdgeId edgeId = RectEdgeId::Top;
    bool ok = false;

    std::vector<CaliperRegion> calipers;
    std::vector<EdgePoint> inputPoints;
    std::vector<EdgePoint> inlierPoints;
    std::vector<EdgePoint> outlierPoints;

    LineModel fittedLine;

    double rmsError = 0.0;
    double score = 0.0;
    QString message;
};

struct VISIONTOOLS_API FindRectResult
{
    bool ok = false;

    RectModel rect;

    FindRectEdgeResult top;
    FindRectEdgeResult bottom;
    FindRectEdgeResult left;
    FindRectEdgeResult right;

    double rmsError = 0.0;
    double maxError = 0.0;
    double score = 0.0;

    QString message;
};

} // namespace VisionTools
