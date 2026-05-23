#pragma once

#include "shape_match/evaluation/PoseErrorEvaluator.h"

namespace ShapeMatch {

class MultiTargetEvaluator
{
public:
    explicit MultiTargetEvaluator(ShapeMatchEvalConfig config = {});

    MultiTargetEvalResult evaluate(const std::vector<ScoredCandidate>& candidates,
                                   const std::vector<GroundTruthInstance>& groundTruth) const;

private:
    bool candidateHitsAnyGt(const ScoredCandidate& candidate,
                            const std::vector<GroundTruthInstance>& groundTruth) const;

    ShapeMatchEvalConfig m_config;
};

} // namespace ShapeMatch
