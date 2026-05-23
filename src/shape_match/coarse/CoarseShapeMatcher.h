#pragma once

#include "shape_match/coarse/CoarseMatchConfig.h"
#include "shape_match/coarse/CoarseMatchReport.h"
#include "shape_match/core/EdgeImageData.h"
#include "shape_match/evaluation/ShapeMatchEvalConfig.h"

namespace ShapeMatch {

class CoarseShapeMatcher
{
public:
    explicit CoarseShapeMatcher(CoarseMatchConfig coarseConfig = {},
                                ShapeMatchEvalConfig evalConfig = {});

    CoarseMatchReport match(const ShapeTemplateModel& templateModel,
                            const EdgeImageData& edgeData,
                            const std::vector<GroundTruthInstance>* groundTruth = nullptr) const;

private:
    CoarseMatchConfig m_coarseConfig;
    ShapeMatchEvalConfig m_evalConfig;
};

} // namespace ShapeMatch
