#pragma once

#include "shape_match/pipeline_v2/ResponsePeakVerifier.h"

namespace ShapeMatch {

class ResponsePeakCandidateSelector
{
public:
    std::vector<VerifiedResponsePeak> select(const std::vector<VerifiedResponsePeak>& verifiedPeaks,
                                             const ShapeMatchPipelineV2Context& context,
                                             const ShapeMatchPipelineV2Config& config,
                                             double* elapsedMs = nullptr) const;
};

} // namespace ShapeMatch
