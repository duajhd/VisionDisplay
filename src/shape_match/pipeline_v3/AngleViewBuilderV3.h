#pragma once

#include "shape_match/pipeline_v3/ShapeMatchV3Types.h"

namespace ShapeMatch {

class AngleViewBuilderV3
{
public:
    std::vector<AngleViewV3> build(const ShapeModelV3& model, float scale, int stride,
                                   float angleStart, float angleExtent, float angleStep,
                                   const ShapeSearchParametersV3& search) const;
    void precompute(ShapeModelV3& model) const;
    std::vector<AngleViewV3> bindPrecomputed(const ShapeModelV3& model,
                                             int pyramidLevel, int stride) const;
};

} // namespace ShapeMatch
