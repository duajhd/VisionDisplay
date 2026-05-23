#pragma once

#include "shape_match/coarse/CoarseMatchReport.h"
#include "shape_match/core/ShapeTemplateModel.h"
#include "shape_match/overlay/ShapeMatchOverlayData.h"

namespace ShapeMatch {

class CoarseMatchDebugOverlay
{
public:
    ShapeMatchOverlayData buildOverlayData(const CoarseMatchReport& report,
                                           const ShapeTemplateModel& model,
                                           int maxCandidates = 10,
                                           bool includeCoarseSeedMarkers = false) const;
};

} // namespace ShapeMatch
