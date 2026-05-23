#pragma once

#include "shape_match/core/ShapeMatchTypes.h"

#include <filesystem>

namespace ShapeMatch {

class MatchReportWriter
{
public:
    bool writeJsonReport(const MatchDiagnosticReport& report,
                         const std::filesystem::path& path) const;
    bool writeCandidatesCsv(const MatchDiagnosticReport& report,
                            const std::filesystem::path& path) const;
    bool writePointEvalCsv(const MatchDiagnosticReport& report,
                           const std::filesystem::path& path) const;
    bool writeSummaryTxt(const MatchDiagnosticReport& report,
                         const std::filesystem::path& path) const;
    bool writeAll(const MatchDiagnosticReport& report,
                  const std::filesystem::path& reportsDir) const;
};

} // namespace ShapeMatch
