#pragma once

#include "shape_match/pipeline_v2/ResponsePeakVerifier.h"

#include <opencv2/core.hpp>

#include <vector>

namespace ShapeMatch {

struct ResponseCandidateRegion
{
    int regionId = -1;
    cv::Rect roiLevel0;
    cv::Rect roiAtResponseLevel;

    MatchPose seedPoseAtResponseLevel;
    MatchPose seedPoseLevel0;

    double seedScore = 0.0;
    int sourcePeakCount = 0;
};

class ResponsePeakRegionGenerator
{
public:
    std::vector<ResponseCandidateRegion> generateRegions(const std::vector<VerifiedResponsePeak>& peaks,
                                                         const ShapeMatchPipelineV2Context& context,
                                                         const ShapeMatchPipelineV2Config& config,
                                                         double* elapsedMs = nullptr) const;
};

} // namespace ShapeMatch
