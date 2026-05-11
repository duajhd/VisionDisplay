#pragma once

#include "VisionTools/VisionTools_global.h"

namespace VisionTools {

struct VISIONTOOLS_API EdgePoint
{
    double x = 0.0;
    double y = 0.0;
    double nx = 0.0;
    double ny = 0.0;
    double response = 0.0;
    double position1D = 0.0;
    double subIndex = 0.0;
    bool valid = false;
};

} // namespace VisionTools
