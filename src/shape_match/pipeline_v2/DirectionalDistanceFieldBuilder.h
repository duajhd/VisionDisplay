#pragma once

#include "shape_match/core/EdgeImageData.h"
#include "shape_match/pipeline_v2/DirectionalDistanceField.h"
#include "shape_match/pipeline_v2/ShapeMatchPipelineV2Config.h"

namespace ShapeMatch {

class DirectionalDistanceFieldBuilder
{
public:
    DirectionalDistanceField build(const EdgeImageData& edgeData,
                                   int level,
                                   const DirectionalChamferConfig& config) const;
};

} // namespace ShapeMatch
