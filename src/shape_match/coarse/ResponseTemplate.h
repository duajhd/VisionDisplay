#pragma once

#include "shape_match/coarse/OrientationResponseConfig.h"
#include "shape_match/core/ShapeTemplateModel.h"

#include <vector>

namespace ShapeMatch {

struct ResponseTemplatePoint
{
    float x = 0.0f;
    float y = 0.0f;

    float gx = 0.0f;
    float gy = 0.0f;

    int orientationBin = 0;
    float weight = 1.0f;

    int pointId = -1;
};

struct ResponseTemplate
{
    int level = -1;
    int orientationBinCount = 0;
    std::vector<ResponseTemplatePoint> points;

    bool empty() const;
    int size() const;
};

class ResponseTemplateBuilder
{
public:
    ResponseTemplate build(const ShapeTemplateModel& model,
                           int level,
                           const OrientationResponseConfig& config) const;
};

} // namespace ShapeMatch
