#pragma once

#include "shape_match/core/ShapeTemplateModel.h"
#include "shape_match/pipeline_v2/ShapeMatchPipelineV2Config.h"

#include <vector>

namespace ShapeMatch {

struct SegmentVerifyPoint
{
    float x = 0.0f;
    float y = 0.0f;
    float gx = 0.0f;
    float gy = 0.0f;
    float weight = 1.0f;
    int orientationBin = 0;
    int pointId = -1;
    int segmentId = -1;
};

struct SegmentTemplate
{
    int level = -1;
    int orientationBinCount = 0;

    std::vector<SegmentVerifyPoint> points;

    struct SegmentInfo {
        int segmentId = -1;
        int start = 0;
        int count = 0;
        float totalWeight = 0.0f;
    };

    std::vector<SegmentInfo> segments;

    bool empty() const;
    int pointCount() const;
    int segmentCount() const;
};

class SegmentTemplateBuilder
{
public:
    SegmentTemplate build(const ShapeTemplateModel& model,
                          int level,
                          const DirectionalChamferConfig& config) const;
};

} // namespace ShapeMatch
