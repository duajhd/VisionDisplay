#pragma once

#include "shape_match/pipeline_v3/ShapeMatchV3Types.h"

namespace ShapeMatch {

class ShapeMatcherV3
{
public:
    ShapeModelV3 createModel(const cv::Mat& templateImage, const cv::Mat& templateMask,
                             const ShapeModelParametersV3& parameters = {}) const;
    std::vector<MatchResultV3> find(const cv::Mat& searchImage, const ShapeModelV3& model,
                                    const ShapeSearchParametersV3& parameters,
                                    ShapeMatchStatisticsV3* statistics = nullptr,
                                    const ICandidateVerifierV3* verifier = nullptr) const;
};

} // namespace ShapeMatch
