#pragma once

#include "VisionTools/VisionTools_global.h"

#include <QString>

namespace VisionTools::Backend {

class VISIONTOOLS_API DependencySelfTest
{
public:
    static QString run();
};

} // namespace VisionTools::Backend
