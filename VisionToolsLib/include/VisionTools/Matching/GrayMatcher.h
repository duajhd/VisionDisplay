#pragma once

#include "VisionTools/Matching/GrayTemplate.h"
#include "VisionTools/Matching/MatchTypes.h"
#include "VisionTools/VisionTools_global.h"

#include <QImage>
#include <QRect>

namespace VisionTools::Matching {

class VISIONTOOLS_API GrayMatcher
{
public:
    GrayTemplate buildTemplate(const QImage& image, QRect roi) const;
    GrayMatchResult match(const QImage& searchImage,
                          const GrayTemplate& tmpl,
                          const GrayMatchParams& params) const;

private:
    static double scoreAt(const std::vector<double>& image,
                          int imageWidth,
                          int imageHeight,
                          const GrayTemplate& tmpl,
                          int x,
                          int y);
};

} // namespace VisionTools::Matching
