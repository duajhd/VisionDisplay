#pragma once

#include "shape_match/coarse/CoarseMatchReport.h"
#include "shape_match/coarse/CoarseMatchConfig.h"

#include <filesystem>

namespace ShapeMatch {

class CoarseMatchReportWriter
{
public:
    bool writeAll(const CoarseMatchReport& report,
                  const CoarseMatchConfig& config,
                  const std::filesystem::path& reportsDir) const;
};

} // namespace ShapeMatch
