#pragma once

#include "shape_match/coarse/OrientationResponseConfig.h"
#include "shape_match/coarse/ResponseDebugReport.h"

#include <filesystem>

namespace ShapeMatch {

class ResponseDebugReportWriter
{
public:
    explicit ResponseDebugReportWriter(std::filesystem::path reportDir = std::filesystem::path("data") / "shape_match" / "reports");

    void write(const ResponseDebugReport& report, const OrientationResponseConfig& config) const;

private:
    std::filesystem::path m_reportDir;
};

} // namespace ShapeMatch
