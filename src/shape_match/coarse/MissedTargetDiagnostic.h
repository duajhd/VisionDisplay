#pragma once

#include "shape_match/coarse/CoarseCandidate.h"
#include "shape_match/coarse/CoarseMatchConfig.h"
#include "shape_match/coarse/ImagePyramid.h"
#include "shape_match/coarse/TemplatePyramid.h"

#include <filesystem>

namespace ShapeMatch {

class MissedTargetDiagnostic
{
public:
    bool writeAll(const CoarseMatchReport& report,
                  const CoarseMatchConfig& config,
                  const ImagePyramid& imagePyramid,
                  const TemplatePyramid& templatePyramid,
                  const std::filesystem::path& reportsDir) const;
};

} // namespace ShapeMatch
