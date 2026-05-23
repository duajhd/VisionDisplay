#pragma once

#include "VisionTools/Matching/ShapeTemplateModel.h"
#include "VisionTools/VisionTools_global.h"

#include <QImage>
#include <QPointF>
#include <QRect>

namespace VisionTools::Matching {

class VISIONTOOLS_API ShapeModelBuilder
{
public:
    ShapeTemplateModel build(const QImage& image,
                             QRect roi,
                             const ShapeModelParams& params = ShapeModelParams(),
                             QPointF origin = QPointF()) const;
    ShapeTemplateModel build(const ShapeModelBuildInput& input,
                             const ShapeModelParams& params = ShapeModelParams(),
                             QPointF origin = QPointF()) const;
};

} // namespace VisionTools::Matching
