#include "VisionDisplay/OverlayData.h"

namespace VisionDisplay {

void VisionDisplayOverlayData::clear()
{
    points.clear();
    polylines.clear();
    arrows.clear();
    texts.clear();
}

bool VisionDisplayOverlayData::empty() const
{
    return points.isEmpty()
        && polylines.isEmpty()
        && arrows.isEmpty()
        && texts.isEmpty();
}

} // namespace VisionDisplay
