#pragma once

#include "VisionTools/VisionTools_global.h"

#include <QString>

#include <cstdint>
#include <vector>

namespace VisionTools::Matching {

struct VISIONTOOLS_API GrayTemplate
{
    std::vector<double> gray;
    std::vector<std::uint8_t> mask;
    int width = 0;
    int height = 0;
    double originX = 0.0;
    double originY = 0.0;
    double mean = 0.0;
    double norm = 0.0;
    QString message;

    bool isValid() const
    {
        return width > 0
            && height > 0
            && gray.size() == static_cast<size_t>(width * height)
            && norm > 0.0;
    }
};

} // namespace VisionTools::Matching
