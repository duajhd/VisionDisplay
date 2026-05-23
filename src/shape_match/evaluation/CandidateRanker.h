#pragma once

#include "shape_match/evaluation/MatchEvaluator.h"
#include "shape_match/evaluation/PoseErrorEvaluator.h"

namespace ShapeMatch {

class CandidateRanker
{
public:
    explicit CandidateRanker(ShapeMatchEvalConfig config = {});

    std::vector<ScoredCandidate> rank(const std::vector<MatchPose>& candidates,
                                      const ShapeTemplateModel& model,
                                      const EdgeImageData& edgeData,
                                      const std::vector<GroundTruthInstance>* groundTruth = nullptr,
                                      FinalRankerProfile* profile = nullptr) const;

    std::vector<ScoredCandidate> rank(const std::vector<MatchPose>& candidates,
                                      const ShapeTemplateModel& model,
                                      const EdgeQueryContext& queryContext,
                                      const std::vector<GroundTruthInstance>* groundTruth = nullptr,
                                      FinalRankerProfile* profile = nullptr) const;

private:
    ShapeMatchEvalConfig m_config;
};

} // namespace ShapeMatch
