#pragma once

#include "shape_match/coarse/OrientationResponseConfig.h"
#include "shape_match/pipeline_v2/MicroAdjustProfile.h"

#include <filesystem>

namespace ShapeMatch {

enum class CandidateGenerationMode {
    OldVoting,
    OrientationResponse,
    CompareOldAndResponse
};

enum class PipelineV2ExecutionMode {
    CandidateOnly,
    VerifyCandidates,
    FullCoarseMatch,
    CompareWithOldPipeline
};

enum class ShapeMatchRuntimeMode {
    Debug,
    Production,
    Benchmark
};

const char* candidateGenerationModeName(CandidateGenerationMode mode);
const char* pipelineV2ExecutionModeName(PipelineV2ExecutionMode mode);
const char* shapeMatchRuntimeModeName(ShapeMatchRuntimeMode mode);

struct PipelineV2ProductionConfig
{
    ShapeMatchRuntimeMode runtimeMode = ShapeMatchRuntimeMode::Debug;

    bool enableRuntimeCaches = true;
    bool enableBufferPool = true;

    bool enableParallelThetaEvaluation = true;
    bool enableParallelChamferVerification = true;
    bool enableParallelFinalRanking = true;

    int numWorkerThreads = 0;

    bool exportDebugImages = true;
    bool exportDebugCsv = true;
    bool exportDebugJson = true;
    bool exportOverlay = true;

    bool productionExportOnlySummary = true;
    bool productionDisablePointEval = true;
    bool productionDisableSegmentCsv = true;
    bool productionDisableResponseImages = true;
    bool productionDisableStageTrace = true;

    int benchmarkWarmupRuns = 3;
    int benchmarkRuns = 20;
};

struct SymmetryConfig
{
    bool enableSymmetrySuppression = true;
    bool assume180DegreeSymmetry = false;
    double symmetryAngleDeg = 180.0;
    double symmetryNmsTranslationPx = 8.0;
    double symmetryNmsAngleToleranceDeg = 5.0;
};

struct ResponsePipelineConfig
{
    bool enableResponsePipeline = true;

    PipelineV2ExecutionMode executionMode = PipelineV2ExecutionMode::FullCoarseMatch;

    int responsePyramidLevel = 3;

    int maxResponsePeaks = 1000;
    int maxVerifiedResponsePeaks = 300;
    int maxFinalCandidates = 80;
    int targetFinalCandidates = 50;

    bool enableResponsePeakVerification = true;
    bool enableResponsePeakNms = true;
    bool enableResponsePeakTileDiversity = true;

    bool enableResponsePeakRegions = true;
    bool enableRoiDistanceFieldForResponseRegions = true;

    bool enableMicroAdjustment = false;
    int microAdjustRadiusPx = 2;
    double microAdjustAngleRadiusDeg = 2.0;
    int maxMicroAdjustChildrenPerPeak = 9;

    bool enableLevel0FinalPoseMicroAdjustment = true;
    MicroAdjustMode microAdjustMode = MicroAdjustMode::Minimal;
    int microAdjustMaxInputCandidates = 12;
    int microAdjustTargetInputCandidates = 12;
    int microAdjustMaxChildrenPerCandidate = 9;
    int microAdjustMaxTotalChildren = 120;
    bool microAdjustUseDirectionalChamfer = true;
    int microAdjustMaxTemplatePoints = 240;
    double microAdjustDistanceSigmaPx = 3.0;
    double microAdjustMaxDistancePx = 12.0;
    double microAdjustOrientationSigmaDeg = 25.0;
    int finalPoseMicroAdjustRadiusPx = 6;
    int finalPoseMicroAdjustStepPx = 2;
    double finalPoseMicroAdjustAngleRadiusDeg = 4.0;
    double finalPoseMicroAdjustAngleStepDeg = 1.0;

    bool enableFinalRanker = true;
    bool enableGtEvaluation = true;
    bool outputOnlyAcceptedFinalCandidates = true;
    int maxOutputCandidates = 32;
    double minOutputFinalScore = 0.45;
    double minOutputScoreRelativeToBest = 0.70;
    double finalOutputNmsPx = 80.0;
    bool finalOutputNmsIgnoreAngle = true;

    bool exportPipelineV2Reports = true;
    bool exportPipelineV2Overlay = true;

    bool compareWithOldPipeline = true;

    double responseWeight = 0.5;
    double fastScoreWeight = 0.5;
    double minVerifiedFastScore = 0.0;

    double responseCandidateNmsPx = 8.0;
    double responseCandidateNmsThetaDeg = 5.0;

    int responseCandidateTileRows = 8;
    int responseCandidateTileCols = 8;
    int maxCandidatesPerTile = 5;

    int regionPaddingPxLevel0 = 32;
    int maxResponseRegions = 80;
    int maxTotalRegionPixels = 4000000;
};

struct DirectionalChamferConfig
{
    bool enableDirectionalChamferVerification = true;

    int orientationBinCount = 16;
    int orientationSpreadRadiusBins = 1;

    bool buildDirectionalDistanceField = true;
    bool useExistingDistanceFieldIfAvailable = true;
    bool useRoiDistanceField = true;

    int maxTemplateVerifyPoints = 600;
    int minTemplateVerifyPoints = 80;

    int maxSegments = 128;
    int minPointsPerSegment = 3;

    double distanceSigmaPx = 3.0;
    double maxDistancePx = 10.0;

    double orientationSigmaDeg = 20.0;
    double maxOrientationErrorDeg = 45.0;

    double wDistance = 0.55;
    double wOrientation = 0.35;
    double wCoverage = 0.10;

    bool enableSegmentScoring = true;
    bool enableOcclusionHandling = true;

    double minSegmentScore = 0.25;
    double minSegmentCoverage = 0.20;
    double minTotalCoverage = 0.20;

    bool enableGreedyPruning = true;
    int pruningCheckInterval = 32;
    double upperBoundMargin = 0.01;
    int minEvaluatedPointsBeforePruning = 64;

    int maxCandidatesToVerify = 300;
    int targetCandidatesAfterVerify = 80;

    bool exportDirectionalChamferReport = true;
    bool exportSegmentDebugCsv = true;
};

struct ShapeMatchPipelineV2Config
{
    CandidateGenerationMode candidateGenerationMode = CandidateGenerationMode::OldVoting;

    int pyramidLevel = 3;
    int maxCandidates = 1000;

    bool enableDebugReport = true;
    bool enableGtEvaluation = true;
    bool enableOverlayData = false;

    OrientationResponseConfig orientationResponseConfig;
    ResponsePipelineConfig responsePipeline;
    DirectionalChamferConfig directionalChamfer;
    PipelineV2ProductionConfig production;
    SymmetryConfig symmetry;

    std::filesystem::path reportDir = std::filesystem::path("data") / "shape_match" / "reports";
};

} // namespace ShapeMatch
