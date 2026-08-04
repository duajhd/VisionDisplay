#pragma once

#include "shape_match/core/ShapeMatchTypes.h"
#include "shape_match/pipeline_v2/ShapeMatchPipelineV2Config.h"

#include <string>
#include <vector>

namespace ShapeMatch {

struct ShapeMatchPipelineV2Result
{
    bool ok = false;
    std::string failureReason;

    CandidateGenerationMode mode = CandidateGenerationMode::OldVoting;

    std::vector<MatchPose> candidates;

    int candidateCount = 0;
    int responsePeakCount = 0;
    int verifiedResponsePeakCount = 0;
    int selectedCandidateCount = 0;
    int candidateRegionCount = 0;

    double candidateGenerationMs = 0.0;
    double responseGenerationMs = 0.0;
    double responseVerificationMs = 0.0;
    double candidateSelectionMs = 0.0;
    double regionGenerationMs = 0.0;
    double roiDistanceFieldBuildMs = 0.0;
    double directionalChamferTimeMs = 0.0;
    double directionalChamferFieldBuildTimeMs = 0.0;
    double microAdjustTotalMs = 0.0;
    double microAdjustGenerateMs = 0.0;
    double microAdjustScoreMs = 0.0;
    double microAdjustNmsMs = 0.0;
    double finalRankerMs = 0.0;
    double reportWriteMs = 0.0;
    double overlayBuildMs = 0.0;
    double unaccountedTimeMs = 0.0;

    int microAdjustSelectedCount = 0;
    int microAdjustGeneratedCount = 0;
    int microAdjustScoredCount = 0;
    int microAdjustInputCandidateCount = 0;
    double microAdjustSetupMs = 0.0;

    int directionalChamferInputCount = 0;
    int directionalChamferAcceptedCount = 0;
    int directionalChamferPrunedCount = 0;

    int responseTemplateCacheHits = 0;
    int responseTemplateCacheMisses = 0;
    int segmentTemplateCacheHits = 0;
    int segmentTemplateCacheMisses = 0;
    int directionalFieldCacheHits = 0;
    int directionalFieldCacheMisses = 0;
    int responseMapCacheHits = 0;
    int responseMapCacheMisses = 0;

    size_t matAllocatedCount = 0;
    size_t matReusedCount = 0;
    size_t vectorAllocatedCount = 0;
    size_t vectorReusedCount = 0;

    int symmetrySuppressedCount = 0;
    double totalTimeMs = 0.0;

    bool usedOldLocalRefine = false;
    bool usedOldBeamPropagation = false;

    int oldLevel0RawChildren = 0;
    int oldLevel0ScoredChildren = 0;

    bool gtEvaluated = false;
    double recall = 0.0;
    double precision = 0.0;
    double f1 = 0.0;
    bool top1Hit = false;
    bool top3Hit = false;
    bool top5Hit = false;
    bool top10Hit = false;
};

} // namespace ShapeMatch
