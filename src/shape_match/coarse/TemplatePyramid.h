#pragma once

#include "shape_match/core/ShapeTemplateModel.h"

#include <vector>

namespace ShapeMatch {

class TemplatePyramid
{
public:
    bool build(const ShapeTemplateModel& baseModel, int levels, int maxPointsPerLevel);

    int levelCount() const;
    const ShapeTemplateModel& level(int levelIndex) const;
    double scaleOfLevel(int levelIndex) const;

private:
    std::vector<ShapeTemplateModel> m_levels;
    std::vector<double> m_scales;
};

} // namespace ShapeMatch
