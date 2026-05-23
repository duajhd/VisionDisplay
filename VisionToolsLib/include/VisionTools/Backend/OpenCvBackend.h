#pragma once

#include "VisionTools/VisionTools_global.h"

#include <QString>

namespace VisionTools::Backend {

class VISIONTOOLS_API OpenCvBackend
{
public:
    static QString openCvBuildInfo();
    static bool runOpenCvSmokeTest(QString& message);
};

} // namespace VisionTools::Backend
