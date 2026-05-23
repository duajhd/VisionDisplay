#pragma once

#include "shape_match/overlay/ShapeMatchOverlayData.h"
#include "VisionDisplay/OverlayData.h"

namespace ShapeMatch {

class VisionDisplayOverlayAdapter
{
public:
    static VisionDisplay::VisionDisplayOverlayData toVisionDisplayOverlay(const ShapeMatchOverlayData& data);
};

} // namespace ShapeMatch
