#pragma once

#include "shape_match/core/ShapeTemplateModel.h"

#include <cstdint>
#include <vector>

namespace ShapeMatch {

struct TemplatePointSoA
{
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> nx;
    std::vector<float> ny;
    std::vector<float> gx;
    std::vector<float> gy;
    std::vector<float> weight;
    std::vector<float> gradMag;
    std::vector<uint8_t> polarity;
    std::vector<int> pointId;
    std::vector<int> orderedIndices;
    float totalWeight = 0.0f;

    bool empty() const;
    int size() const;
};

class TemplatePointSoABuilder
{
public:
    TemplatePointSoA build(const ShapeTemplateModel& model, bool enablePointOrdering) const;
};

float templatePointWeight(float weight, float gradMag);

} // namespace ShapeMatch
