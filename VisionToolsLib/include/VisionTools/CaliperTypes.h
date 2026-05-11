#pragma once

#include "VisionTools/EdgePoint.h"
#include "VisionTools/VisionTools_global.h"

#include <QString>

#include <vector>

namespace VisionTools {

enum class EdgePolarity
{
    DarkToLight,
    LightToDark,
    Any
};

enum class EdgeSelection
{
    First,
    Last,
    Strongest,
    NearestToCenter,
    NearestToExpected,
    All
};

struct VISIONTOOLS_API CaliperRegion
{
    double centerX = 0.0;
    double centerY = 0.0;
    double length = 0.0;
    double width = 0.0;
    double angleDeg = 0.0;
};

struct VISIONTOOLS_API CaliperParams
{
    int sampleCount = 101;
    int projectionCount = 5;
    double smoothingSigma = 1.0;
    double minResponse = 5.0;
    EdgePolarity polarity = EdgePolarity::Any;
    EdgeSelection selection = EdgeSelection::Strongest;
    EdgeSelection fallbackSelection = EdgeSelection::Strongest;
    double expectedPosition1D = 0.0;
    double maxPositionDeviation = 10.0;
    bool enableSubpixel = true;
};

struct VISIONTOOLS_API CaliperResult
{
    bool ok = false;
    std::vector<double> profile;
    std::vector<double> smoothedProfile;
    std::vector<double> gradient;
    std::vector<EdgePoint> candidates;
    std::vector<EdgePoint> selectedEdges;
    QString message;
};

} // namespace VisionTools
