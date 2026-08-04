#pragma once

#include "shape_match/pipeline_v3/ShapeMatchV3Types.h"

namespace ShapeMatch {

class ShapeTemplateModel;
class WorkerPoolV3;

class ResponseMapBuilderV3
{
public:
    ResponseMapV3 build(const cv::Mat& image, float low, float high,
                        bool nonMaximumSuppression,
                        WorkerPoolV3* workerPool = nullptr) const;
};

class ShapeModelTrainerV3
{
public:
    ShapeModelV3 createModel(const cv::Mat& templateImage, const cv::Mat& templateMask,
                             const ShapeModelParametersV3& parameters) const;
    ShapeModelV3 fromTemplateModel(const ShapeTemplateModel& model,
                                   const ShapeModelParametersV3& parameters) const;
};

} // namespace ShapeMatch
