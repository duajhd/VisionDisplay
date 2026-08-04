#pragma once

#include "shape_match/core/ShapeMatchTypes.h"

#include <opencv2/core.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

namespace ShapeMatch {

inline constexpr int kOrientationBinCountV3 = 18; // compatibility/default
inline constexpr int kMaxOrientationBinCountV3 = 36;
inline constexpr int kPyramidLevelCountV3 = 4;
inline constexpr int kPrecomputedAngleCountV3 = 36;

enum class SearchSafetyV3 { Safe, Balanced, Fast };

struct ShapePointV3
{
    float x = 0.0f;
    float y = 0.0f;
    float quality = 0.0f;
    float normalX = 1.0f;
    float normalY = 0.0f;
    float curvature = 0.0f;
    float gradientAngleRadians = 0.0f;
    std::uint16_t contourId = 0;
    std::uint16_t partId = 0;
    std::uint8_t orientationBin = 0;
    std::uint8_t polarity = 0;
    std::uint8_t weight = 1;
    std::uint8_t stage = 0;
};

struct RotatedPointV3
{
    std::int16_t dx = 0;
    std::int16_t dy = 0;
    std::int32_t linearOffset = 0;
    std::uint8_t orientationBin = 0;
    std::uint8_t weight = 1;
    std::uint16_t partId = 0;
};

struct ScoreStageV3
{
    std::uint32_t begin = 0;
    std::uint32_t end = 0;
    std::uint32_t cumulativeWeight = 0;
    std::uint32_t remainingWeight = 0;
    float greedyMaxMeanDistance = 0.0f;
};

struct AngleViewV3
{
    float angleRadians = 0.0f;
    std::vector<RotatedPointV3> points;
    std::vector<ScoreStageV3> stages;
    std::uint32_t totalWeight = 0;
    int minDx = 0;
    int maxDx = 0;
    int minDy = 0;
    int maxDy = 0;
};

struct PyramidLevelModelV3
{
    float scale = 1.0f;
    int stride = 0;
    std::vector<AngleViewV3> angleViews;
};

struct ResponseMapV3
{
    // Plane-major uint8 response costs: 0 is a hit, 255 is no response.
    std::array<cv::Mat, kMaxOrientationBinCountV3> binMaps;
    std::array<const std::uint8_t*, kMaxOrientationBinCountV3> binData {};
    int width = 0;
    int height = 0;
    int stride = 0;
    int planeStride = 0;
    int orientationBinCount = 1;
    float maxDistancePx = 8.0f;
    float distanceScale = 255.0f / 8.0f;

    bool valid() const
    {
        if (width <= 0 || height <= 0 || stride < width) return false;
        if (orientationBinCount <= 0 || orientationBinCount > kMaxOrientationBinCountV3) return false;
        for (int i = 0; i < orientationBinCount; ++i) if (!binData[static_cast<size_t>(i)]) return false;
        return true;
    }
};

struct MatchCandidateV3
{
    int x = 0;
    int y = 0;
    int angleIndex = 0;
    int pyramidLevel = 0;
    std::uint32_t rawCost = 0;
    float score = 0.0f;
};

struct ScoreBlock16V3
{
    alignas(32) std::array<std::uint32_t, 16> rawCosts {};
    std::uint16_t validMask = 0;
    std::array<std::uint8_t, 16> rejectedStage {};
};

struct ScoreBlock32V3
{
    alignas(32) std::array<std::uint32_t, 32> rawCosts {};
    std::uint32_t validMask = 0;
};

struct ScoreBlock64V3
{
    alignas(64) std::array<std::uint32_t, 64> rawCosts {};
    std::uint64_t validMask = 0;
};

struct ShapeModelParametersV3
{
    float gradientLow = 20.0f;
    float gradientHigh = 100.0f;
    bool nonMaximumSuppression = false;
    int responseSpreadRadius = 0;
    float distanceFieldMaxDistancePx = 8.0f;
    float templateGradientThreshold = 30.0f;
    float templatePointMinDistance = 2.0f;
    int pyramidLevels = 4;
    int stage0PointCount = 48;
    int stage1PointCount = 128;
    int stage2PointCount = 256;
    float angleStartRadians = 0.0f;
    float angleExtentRadians = 0.0f;
    float coarseAngleStepRadians = 0.0f;
};

struct ShapeSearchParametersV3
{
    cv::Rect searchRoi;
    float angleStartRadians = 0.0f;
    float angleExtentRadians = 0.0f;
    float angleStepRadians = 0.0f;
    float minScore = 0.5f;
    int pyramidLevels = 0;
    int maxMatches = 1;
    int coarseTopK = 100;
    SearchSafetyV3 safetyMode = SearchSafetyV3::Safe;
    bool enableAvx2 = true;
    bool enableMultithreading = true;
    bool enableFinalVerification = true;
    bool enableSubpixelRefinement = true;
    bool enablePyramidRefinement = false;
    bool enableStatistics = true;
    int numThreads = 4;
    int tileWidth = 32;
    int tileHeight = 32;
    int stage0PointCount = 48;
    int stage1PointCount = 128;
    int stage2PointCount = 256;
    int refineRadius = 4;
    int refineAngleRadius = 1;
    int maxRefineCandidates = 40;
    int subpixelMaxIterations = 12;
    int subpixelMinCorrespondences = 24;
    float subpixelSearchRadius = 4.0f;
    float subpixelHuberDelta = 1.5f;
    float subpixelOrientationToleranceRadians = 0.5235988f;
    float subpixelMaxTranslationStep = 2.0f;
    float subpixelMaxAngleStepRadians = 0.0349066f;
    float refinedMinVisibleRatio = 0.65f;
    float refinedMinCorrespondenceRatio = 0.50f;
    float refinedMaxRmsResidual = 1.50f;
    float refinedNmsTemplateFraction = 0.35f;
    float nmsDistance = 8.0f;
    float nmsAngleRadians = 0.08726646f;
    std::array<float, 3> balancedGreedyMean {190.0f, 175.0f, 160.0f};
    std::array<float, 3> fastGreedyMean {150.0f, 135.0f, 120.0f};
};

struct ShapeMatchStatisticsV3
{
    double pyramidTimeMs = 0.0;
    double edgeTimeMs = 0.0;
    double distanceFieldTimeMs = 0.0;
    double responseMapTimeMs = 0.0;
    double angleViewTimeMs = 0.0;
    double coarseSearchTimeMs = 0.0;
    double pyramidTrackTimeMs = 0.0;
    double verifyTimeMs = 0.0;
    double refineTimeMs = 0.0;
    double totalTimeMs = 0.0;
    std::uint64_t evaluatedBlocks = 0;
    std::uint64_t evaluatedCandidates = 0;
    std::uint64_t rejectedAtStage0 = 0;
    std::uint64_t rejectedAtStage1 = 0;
    std::uint64_t rejectedAtStage2 = 0;
    std::uint64_t fullyEvaluated = 0;
    std::uint64_t scalarBlocks = 0;
    std::uint64_t avx2Blocks = 0;
    std::uint64_t avx512Blocks = 0;
    std::uint64_t coarseCandidates = 0;
    std::uint64_t candidatesAfterNms = 0;
    std::uint64_t verifiedCandidates = 0;
};

struct ShapeModelV3
{
    cv::Size templateSize;
    cv::Point2f origin;
    ShapeModelParametersV3 parameters;
    std::vector<ShapePointV3> points;
    std::array<std::vector<AngleViewV3>, kPyramidLevelCountV3> precomputedAngleViews;

    bool empty() const { return points.empty(); }
    bool hasPrecomputedViews() const
    {
        return precomputedAngleViews[3].size() == kPrecomputedAngleCountV3;
    }
};

struct MatchResultV3
{
    MatchPose pose;
    float score = 0.0f;
    std::uint32_t rawCost = 0;
    bool refined = false;
    int refinementIterations = 0;
    int validCorrespondences = 0;
    float visibleRatio = 0.0f;
    float rmsResidual = 0.0f;
};

class ICandidateVerifierV3
{
public:
    virtual ~ICandidateVerifierV3() = default;
    virtual bool verify(const MatchCandidateV3& candidate, MatchResultV3& result) const = 0;
};

} // namespace ShapeMatch
