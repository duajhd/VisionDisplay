#pragma once

#include "shape_match/coarse/MissedTargetStageTrace.h"
#include "shape_match/core/ShapeMatchTypes.h"

#include <string>
#include <vector>

namespace ShapeMatch {

struct CoarseCandidate
{
    int traceCandidateId = -1;
    MatchPose pose;
    std::string source = "grid";
    double voteScore = 0.0;
    int voteCount = 0;
    double fastScore = 0.0;
    double propagationScore = 0.0;
    double coverageApprox = 0.0;
    double orientationApprox = 0.0;
    double polarityApprox = 0.0;

    int pyramidLevel = -1;
    int evaluatedPoints = 0;
    int matchedPoints = 0;
    int totalPointCount = 0;
    bool rejectedByEarlyExit = false;
    bool suppressedByNms = false;
    int suppressorCandidateId = -1;
    std::string rejectReason;
    int sourceCellRow = -1;
    int sourceCellCol = -1;
};

inline double candidateBeamScore(const CoarseCandidate& candidate)
{
    return candidate.propagationScore > 0.0 ? candidate.propagationScore : candidate.fastScore;
}

struct CoarseMatchAngleStat
{
    double angleDeg = 0.0;
    int evaluatedCandidateCount = 0;
    int keptCandidateCount = 0;
    double bestFastScore = 0.0;
    double elapsedMs = 0.0;
};

struct CoarseMatchCellStat
{
    int row = 0;
    int col = 0;
    int candidateCount = 0;
    int keptCount = 0;
};

struct CoarseMatchLevelResult
{
    int level = -1;
    int imageWidth = 0;
    int imageHeight = 0;
    int templatePoints = 0;
    double angleStepDeg = 0.0;
    int translationStepPx = 0;
    int evaluatedCandidateCount = 0;
    int generatedCandidateCount = 0;
    int inputBeamCount = 0;
    int earlyExitCandidateCount = 0;
    int upperBoundPrunedCount = 0;
    int localSearchFallbackCount = 0;
    int fullEvaluatedCandidateCount = 0;
    int prunedCandidateCount = 0;
    long long totalEvaluatedPoints = 0;
    int nmsBeforeCount = 0;
    int nmsAfterCount = 0;
    int keptCandidateCount = 0;
    int rawGeneratedCandidates = 0;
    int afterPreFilterCandidates = 0;
    int afterBudgetCandidates = 0;
    int scoredCandidates = 0;
    int parentCountBeforeSelection = 0;
    int parentCountAfterSelection = 0;
    int budgetClippedCount = 0;
    int level0ParentBeforeV2 = 0;
    int level0ParentAfterV2 = 0;
    int level0RawChildrenBeforeV2 = 0;
    int level0ChildrenAfterRegionFilter = 0;
    int level0ChildrenAfterDuplicateFilter = 0;
    int level0ChildrenAfterBudgetV2 = 0;
    int level0RegionRejectedChildren = 0;
    int level0DuplicateRejectedChildren = 0;
    int maxChildrenPerParent = 0;
    double avgChildrenPerParent = 0.0;
    double candidateReductionRatio = 1.0;
    double bestFastScore = 0.0;
    double worstKeptFastScore = 0.0;
    bool candidateLimitHit = false;
    double elapsedMs = 0.0;
    double fastScoreTimeMs = 0.0;
    double nmsTimeMs = 0.0;
    std::string scoringMode;
    bool distanceFieldValid = false;
    double distanceFieldBuildTimeMs = 0.0;
    double avgEvaluatedPointsPerCandidate = 0.0;
    double soaBuildTimeMs = 0.0;
    double rotatedCacheBuildTimeMs = 0.0;
    double rotatedCacheHitRate = 0.0;
    int rotatedCacheHitCount = 0;
    int rotatedCacheFallbackCount = 0;
    std::string scorerMode = "old";
    double parallelScoringTimeMs = 0.0;
    int numScoringThreads = 1;
    double mergeTimeMs = 0.0;
    std::string candidateSource = "grid";
    int votingRawCandidateCount = 0;
    int votingScoredCandidateCount = 0;
    int votingAcceptedCandidateCount = 0;
    double votingBestScore = 0.0;
    double votingTimeMs = 0.0;
    double votingVerifyTimeMs = 0.0;
    bool gridFallbackUsed = false;
    double buildRtTableMs = 0.0;
    double sampleImageEdgesMs = 0.0;
    double votingAccumulatorMs = 0.0;
    double peakFindMs = 0.0;
    std::vector<CoarseMatchAngleStat> angleStats;
    std::vector<CoarseMatchCellStat> cellStats;
    std::vector<CoarseCandidate> topCandidates;
    std::vector<StageTraceSnapshot> stageTrace;
};

struct CoarseMatchProfile
{
    double totalTimeMs = 0.0;
    double pyramidBuildTimeMs = 0.0;
    double templatePyramidBuildTimeMs = 0.0;
    double distanceFieldBuildTimeMs = 0.0;
    int upperBoundPrunedCount = 0;
    long long totalEvaluatedPoints = 0;
    int totalEvaluatedCandidates = 0;
    double avgEvaluatedPointsPerCandidate = 0.0;
    double globalSearchTimeMs = 0.0;
    double votingTimeMs = 0.0;
    double buildRtTableMs = 0.0;
    double sampleImageEdgesMs = 0.0;
    double votingAccumulatorMs = 0.0;
    double peakFindMs = 0.0;
    double votingVerifyMs = 0.0;
    double localRefineTimeMs = 0.0;
    double fastScoreTimeMs = 0.0;
    double nmsTimeMs = 0.0;
    double finalRankerTimeMs = 0.0;
    FinalRankerProfile finalRankerProfile;
    bool largeImageMode = false;
    bool skippedFullLevel0DistanceField = false;
    double roiDistanceFieldBuildTimeMs = 0.0;
    int roiDistanceFieldCount = 0;
    int roiDistanceFieldTotalPixels = 0;
    int level0ParentBefore = 0;
    int level0ParentAfter = 0;
    int level0RawChildren = 0;
    int level0ScoredChildren = 0;
    double reportWriteTimeMs = 0.0;
    double overlayBuildTimeMs = 0.0;
    double candidateReductionRatio = 1.0;
    int level0CandidateCountBefore = 0;
    int level0CandidateCountAfterBudget = 0;
    int level0ScoredCandidateCount = 0;
    double soaBuildTimeMs = 0.0;
    double rotatedCacheBuildTimeMs = 0.0;
    double parallelScoringTimeMs = 0.0;
    double mergeTimeMs = 0.0;
};

struct RoiDistanceFieldProfileRow
{
    int level = 0;
    int roiId = -1;
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    int pixelCount = 0;
    double buildTimeMs = 0.0;
    bool valid = false;
    int sourceCandidateCount = 0;
};

struct CoarseMatchReport
{
    std::vector<CoarseMatchLevelResult> levels;
    std::vector<MatchPose> finalPoses;
    std::vector<CoarseCandidate> finalCoarseCandidates;
    std::vector<ScoredCandidate> finalRankedCandidates;
    std::vector<GroundTruthInstance> groundTruthInstances;
    MultiTargetEvalResult multiTargetEval;
    CoarseMatchProfile profile;
    std::vector<RoiDistanceFieldProfileRow> roiDistanceFieldRows;
    std::string initialCandidateMode = "grid";
    int votingRawCandidateCount = 0;
    int votingScoredCandidateCount = 0;
    double votingBestScore = 0.0;
    double votingTimeMs = 0.0;
    bool gridFallbackUsed = false;
    double totalTimeMs = 0.0;
    std::string failureReason;
};

} // namespace ShapeMatch
