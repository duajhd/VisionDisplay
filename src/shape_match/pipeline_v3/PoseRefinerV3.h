#pragma once

#include "shape_match/pipeline_v3/ShapeMatchV3Types.h"

namespace ShapeMatch {

struct FineEdgeMapV3
{
    cv::Mat gx;
    cv::Mat gy;
    cv::Mat magnitude;
};

class PoseRefinerV3
{
public:
    FineEdgeMapV3 buildEdgeMap(const cv::Mat& searchImage) const;
    MatchResultV3 refine(const FineEdgeMapV3& edgeMap,
                         const ShapeModelV3& model,
                         const MatchResultV3& initial,
                         const ShapeSearchParametersV3& parameters) const;
    MatchResultV3 refine(const cv::Mat& searchImage,
                         const ShapeModelV3& model,
                         const MatchResultV3& initial,
                         const ShapeSearchParametersV3& parameters) const;
};

} // namespace ShapeMatch
