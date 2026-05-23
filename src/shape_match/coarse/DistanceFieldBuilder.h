#pragma once

#include "shape_match/coarse/CoarseMatchConfig.h"
#include "shape_match/core/EdgeImageData.h"

namespace ShapeMatch {

class DistanceFieldBuilder
{
public:
    bool build(EdgeImageData& edgeData, const CoarseMatchConfig& config) const;
};

} // namespace ShapeMatch
