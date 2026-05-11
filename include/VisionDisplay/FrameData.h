#pragma once

#include "VisionDisplay/VisionDisplay_global.h"

#include <QImage>

namespace VisionDisplay {

struct VISIONDISPLAY_API FrameData
{
    QImage image;
    quint64 sequence = 0;

    bool isValid() const
    {
        return !image.isNull();
    }
};

} // namespace VisionDisplay
