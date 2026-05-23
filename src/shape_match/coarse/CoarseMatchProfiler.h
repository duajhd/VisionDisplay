#pragma once

#include "shape_match/coarse/CoarseCandidate.h"
#include "shape_match/coarse/CoarseMatchConfig.h"

#include <filesystem>

namespace ShapeMatch {

class CoarseMatchProfiler
{
public:
    bool writeAll(const CoarseMatchReport& report,
                  const CoarseMatchConfig& config,
                  const std::filesystem::path& reportsDir) const;
};

} // namespace ShapeMatch
