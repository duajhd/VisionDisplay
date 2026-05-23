#pragma once

#include "shape_match/core/ShapeTemplateModel.h"
#include "shape_match/evaluation/ShapeMatchEvalConfig.h"

namespace ShapeMatch {

class PoseErrorEvaluator
{
public:
    explicit PoseErrorEvaluator(ShapeMatchEvalConfig config = {});

    PoseError evaluatePoseError(const MatchPose& predicted, const MatchPose& groundTruth) const;
    ContourReprojectionError evaluateContourReprojectionError(const ShapeTemplateModel& model,
                                                              const MatchPose& predicted,
                                                              const MatchPose& groundTruth) const;

private:
    ShapeMatchEvalConfig m_config;
};

} // namespace ShapeMatch
