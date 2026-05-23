#pragma once

#include "VisionTools/VisionTools_global.h"

#include <QString>

namespace VisionTools::Backend {

class VISIONTOOLS_API CeresBackend
{
public:
    static QString ceresBuildInfo();
    static bool runCeresSmokeTest(QString& message);
};

} // namespace VisionTools::Backend
