#pragma once

#include "shape_match/coarse/ResponsePeakExtractor.h"
#include "shape_match/pipeline_v2/ShapeMatchPipelineV2Config.h"
#include "shape_match/pipeline_v2/ShapeMatchPipelineV2Context.h"

#include <string>
#include <vector>

namespace ShapeMatch {

struct VerifiedResponsePeak
{
    ResponsePeak peak;
    MatchPose pose;

    double responseScore = 0.0;
    double fastScore = 0.0;
    double coverageApprox = 0.0;
    double orientationApprox = 0.0;
    double polarityApprox = 0.0;
    double combinedScore = 0.0;

    bool accepted = false;
    std::string rejectReason;
};

class ResponsePeakVerifier
{
public:
    std::vector<VerifiedResponsePeak> verify(const std::vector<ResponsePeak>& peaks,
                                             const ShapeMatchPipelineV2Context& context,
                                             const ShapeMatchPipelineV2Config& config,
                                             double* elapsedMs = nullptr) const;
};

} // namespace ShapeMatch
