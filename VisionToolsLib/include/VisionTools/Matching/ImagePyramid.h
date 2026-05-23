#pragma once

#include "VisionTools/VisionTools_global.h"

#include <QImage>

#include <vector>

namespace VisionTools::Matching {

struct VISIONTOOLS_API GrayImageLevel
{
    std::vector<double> pixels;
    int width = 0;
    int height = 0;
    double scale = 1.0;

    bool isValid() const
    {
        return width > 0
            && height > 0
            && pixels.size() == static_cast<size_t>(width * height);
    }
};

class VISIONTOOLS_API ImagePyramid
{
public:
    static GrayImageLevel fromQImage(const QImage& image);
    static std::vector<GrayImageLevel> build(const QImage& image, int levels);
};

} // namespace VisionTools::Matching
