#pragma once

#include "shape_match/pipeline_v2/DirectionalChamferVerifier.h"

namespace ShapeMatch {

class ParallelDirectionalChamferVerifier
{
public:
    explicit ParallelDirectionalChamferVerifier(const PipelineV2ProductionConfig& productionConfig);

    std::vector<VerifiedResponsePeak> verify(const std::vector<VerifiedResponsePeak>& input,
                                             const SegmentTemplate& templ,
                                             const DirectionalDistanceField& field,
                                             const DirectionalChamferVerifier& verifier,
                                             int targetOutputCount,
                                             int* prunedCount = nullptr) const;

private:
    PipelineV2ProductionConfig m_config;
};

} // namespace ShapeMatch
