#pragma once

#include "shape_match/core/EdgeImageData.h"
#include "shape_match/core/ShapeMatchTypes.h"
#include "shape_match/core/ShapeTemplateModel.h"
#include "shape_match/coarse/ImagePyramid.h"
#include "shape_match/coarse/TemplatePyramid.h"

#include <string>
#include <vector>

namespace ShapeMatch {

struct ShapeMatchPipelineV2Context
{
    const ShapeTemplateModel* templateModel = nullptr;
    const EdgeImageData* edgeData = nullptr;

    const ImagePyramid* imagePyramid = nullptr;
    const TemplatePyramid* templatePyramid = nullptr;

    std::vector<GroundTruthInstance> groundTruth;

    std::string imageName;
    std::string templateName;
};

} // namespace ShapeMatch
