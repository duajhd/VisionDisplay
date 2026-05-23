#pragma once

#include "shape_match/core/ShapeTemplateModel.h"
#include "shape_match/overlay/ShapeMatchOverlayData.h"

namespace ShapeMatch {

class MatchDebugOverlayAdapter
{
public:
    ShapeMatchOverlayData buildOverlayData(const MatchDiagnosticReport& report,
                                           const ShapeTemplateModel& model) const;
};

} // namespace ShapeMatch
