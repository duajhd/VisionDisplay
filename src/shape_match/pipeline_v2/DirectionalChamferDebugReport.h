#pragma once

#include "shape_match/pipeline_v2/DirectionalChamferScore.h"
#include "shape_match/pipeline_v2/ResponsePeakVerifier.h"

#include <vector>

namespace ShapeMatch {

struct DirectionalChamferCandidateDebug
{
    VerifiedResponsePeak candidate;
    DirectionalChamferScore score;
};

struct DirectionalChamferDebugReport
{
    int inputCandidates = 0;
    int acceptedCandidates = 0;
    int prunedCandidates = 0;
    int finalCandidatesToRanker = 0;

    double fieldBuildTimeMs = 0.0;
    double verifyTimeMs = 0.0;
    double avgTimePerCandidateMs = 0.0;
    double topScore = 0.0;

    bool usedOldLocalRefine = false;

    std::vector<DirectionalChamferCandidateDebug> candidates;
};

} // namespace ShapeMatch
