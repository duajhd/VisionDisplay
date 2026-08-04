#include "shape_match/coarse/OrientationResponseMap.h"

namespace ShapeMatch {

bool OrientationResponseMap::empty() const
{
    return !valid || imageSize.empty() || binMaps.empty();
}

} // namespace ShapeMatch
