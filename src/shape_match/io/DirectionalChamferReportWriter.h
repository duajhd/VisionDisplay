#pragma once

#include "shape_match/pipeline_v2/DirectionalChamferDebugReport.h"

#include <filesystem>

namespace ShapeMatch {

class DirectionalChamferReportWriter
{
public:
    explicit DirectionalChamferReportWriter(std::filesystem::path reportDir = std::filesystem::path("data") / "shape_match" / "reports");

    void write(const DirectionalChamferDebugReport& report) const;

private:
    std::filesystem::path m_reportDir;
};

} // namespace ShapeMatch
