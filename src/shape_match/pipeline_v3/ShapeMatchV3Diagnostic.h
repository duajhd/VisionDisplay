#pragma once

#include "shape_match/core/ShapeMatchTypes.h"
#include "shape_match/pipeline_v3/ShapeMatchV3Types.h"

#include <array>
#include <filesystem>
#include <string>
#include <vector>

namespace ShapeMatch {

struct GtLevelProbeV3
{
    int level = 0;
    int x = 0;
    int y = 0;
    int angleIndex = 0;
    double sampledAngleDeg = 0.0;
    double angleQuantizationErrorDeg = 0.0;
    int matchedPoints = 0;
    int totalPoints = 0;
    double score = 0.0;
    int rejectedStage = 0;
    bool insideSearchRoi = true;
    bool insideValidBounds = true;
    bool hasNearbyBestCandidate = false;
    int nearbyBestX = 0;
    int nearbyBestY = 0;
    int nearbyBestAngleIndex = -1;
    double nearbyBestAngleDeg = 0.0;
    double nearbyBestScore = 0.0;
    std::uint32_t nearbyBestRawCost = 0;
    std::uint64_t nearbyBestGlobalRank = 0;
    bool enteredGlobalTop300 = false;
    bool enteredNmsTop40 = false;
    bool suppressedByNms = false;
    bool droppedByNmsCapacity = false;
    int suppressorX = 0;
    int suppressorY = 0;
    int suppressorAngleIndex = -1;
    double suppressorAngleDeg = 0.0;
    double suppressorScore = 0.0;
    std::array<int, kMaxOrientationBinCountV3> expectedPerBin {};
    std::array<int, kMaxOrientationBinCountV3> matchedPerBin {};
};

struct GtDiagnosticV3
{
    GroundTruthInstance groundTruth;
    bool detected = false;
    int nearestResultIndex = -1;
    double nearestResultScore = 0.0;
    double positionErrorPx = 0.0;
    double angleErrorDeg = 0.0;
    std::string missedReason;
    std::vector<GtLevelProbeV3> levels;
};

struct ShapeMatchV3DiagnosticReport
{
    std::string imageName;
    std::string templateName;
    ShapeSearchParametersV3 searchParameters;
    ShapeMatchStatisticsV3 statistics;
    int orientationBinCount = 1;
    int resultCount = 0;
    int gtCount = 0;
    int detectedCount = 0;
    std::vector<MatchResultV3> results;
    std::vector<GtDiagnosticV3> groundTruth;
};

class ShapeMatchV3DiagnosticAnalyzer
{
public:
    ShapeMatchV3DiagnosticReport analyze(const cv::Mat& searchImage,
                                         const ShapeModelV3& model,
                                         const ShapeSearchParametersV3& parameters,
                                         const ShapeMatchStatisticsV3& statistics,
                                         const std::vector<MatchResultV3>& results,
                                         const std::vector<GroundTruthInstance>& groundTruth) const;
};

class ShapeMatchV3ReportWriter
{
public:
    bool write(const ShapeMatchV3DiagnosticReport& report,
               const std::filesystem::path& reportDirectory) const;
};

} // namespace ShapeMatch
