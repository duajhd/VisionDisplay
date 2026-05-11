#pragma once

#include "VisionDisplay/VisionDisplay_global.h"

#include <QObject>

namespace VisionDisplay {
Q_NAMESPACE_EXPORT(VISIONDISPLAY_API)

enum class PixelFormat {
    Gray8 = 0,
    RGB888 = 1,
    BGR888 = 2,
    RGBA8888 = 3,
    BGRA8888 = 4
};
Q_ENUM_NS(PixelFormat)

} // namespace VisionDisplay
