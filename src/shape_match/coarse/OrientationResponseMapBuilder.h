#pragma once

#include "shape_match/coarse/OrientationResponseConfig.h"
#include "shape_match/coarse/OrientationResponseMap.h"
#include "shape_match/core/EdgeImageData.h"

namespace ShapeMatch {

class OrientationResponseMapBuilder
{
public:
    OrientationResponseMap build(const EdgeImageData& edgeData,
                                 int level,
                                 const OrientationResponseConfig& config) const;
};

} // namespace ShapeMatch
