#pragma once

#include "shape_match/core/EdgeImageData.h"
#include "shape_match/coarse/RoiDistanceField.h"
#include "shape_match/evaluation/ShapeMatchEvalConfig.h"

namespace ShapeMatch {

class MatchEvaluator
{
public:
    explicit MatchEvaluator(ShapeMatchEvalConfig config = {});

    MatchScore evaluate(const ShapeTemplateModel& model,
                        const EdgeImageData& edgeData,
                        const MatchPose& pose,
                        std::vector<PointEval>* pointEvals = nullptr,
                        MatchEvaluationStats* stats = nullptr) const;

    MatchScore evaluate(const ShapeTemplateModel& model,
                        const EdgeQueryContext& queryContext,
                        const MatchPose& pose,
                        std::vector<PointEval>* pointEvals = nullptr,
                        MatchEvaluationStats* stats = nullptr) const;

private:
    PointEval evaluatePoint(const TemplatePoint& pt,
                            const EdgeImageData& edgeData,
                            const MatchPose& pose,
                            MatchEvaluationStats* stats) const;
    PointEval evaluatePoint(const TemplatePoint& pt,
                            const EdgeQueryContext& queryContext,
                            const MatchPose& pose,
                            MatchEvaluationStats* stats) const;
    double computeDistanceScore(double distance, double sigma) const;
    double computeOrientationScore(double angleDiffDeg) const;
    double computePolarityScore(bool polarityOk, EdgePolarity polarity) const;
    void computeAggregateStats(MatchScore& score, const std::vector<PointEval>& evals) const;
    bool determineAccepted(MatchScore& score) const;

    ShapeMatchEvalConfig m_config;
};

} // namespace ShapeMatch
