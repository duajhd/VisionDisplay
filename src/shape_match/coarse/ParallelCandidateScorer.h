#pragma once

#include "shape_match/coarse/FastPoseScorer.h"
#include "shape_match/coarse/RotatedTemplateCache.h"

#include <vector>

namespace ShapeMatch {

struct ParallelScoringResult
{
    std::vector<CoarseCandidate> candidates;
    double scoringTimeMs = 0.0;
    double mergeTimeMs = 0.0;
    int numThreads = 1;
    std::vector<int> candidatesPerThread;
};

class ParallelCandidateScorer
{
public:
    explicit ParallelCandidateScorer(CoarseMatchConfig config = {});

    ParallelScoringResult scoreCandidates(const std::vector<MatchPose>& poses,
                                          const TemplatePointSoA& points,
                                          const RotatedTemplateCache& rotationCache,
                                          const EdgeImageData& imageLevel,
                                          int pyramidLevel,
                                          int topK) const;

    ParallelScoringResult scoreCandidates(const std::vector<MatchPose>& poses,
                                          const TemplatePointSoA& points,
                                          const RotatedTemplateCache& rotationCache,
                                          const EdgeQueryContext& queryContext,
                                          int pyramidLevel,
                                          int topK) const;

private:
    int threadCountFor(int candidateCount) const;

    CoarseMatchConfig m_config;
};

} // namespace ShapeMatch
