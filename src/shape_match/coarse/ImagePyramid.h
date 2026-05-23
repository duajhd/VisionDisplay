#pragma once

#include "shape_match/core/EdgeImageData.h"
#include "shape_match/coarse/CoarseMatchConfig.h"

#include <vector>

namespace ShapeMatch {

class ImagePyramid
{
public:
    bool build(const EdgeImageData& baseEdgeData, int levels, const CoarseMatchConfig& config = {});

    int levelCount() const;
    const EdgeImageData& level(int levelIndex) const;
    double scaleOfLevel(int levelIndex) const;
    double distanceFieldBuildTimeMs(int levelIndex) const;
    double totalDistanceFieldBuildTimeMs() const;
    bool skippedFullDistanceField(int levelIndex) const;

private:
    std::vector<EdgeImageData> m_levels;
    std::vector<double> m_scales;
    std::vector<double> m_distanceFieldBuildTimesMs;
    std::vector<bool> m_skippedDistanceField;
};

} // namespace ShapeMatch
