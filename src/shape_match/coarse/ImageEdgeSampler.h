#pragma once

#include "shape_match/coarse/VotingConfig.h"
#include "shape_match/core/EdgeImageData.h"

#include <cstdint>
#include <vector>

namespace ShapeMatch {

struct ImageVotePoint
{
    float x = 0.0f;
    float y = 0.0f;
    float gx = 1.0f;
    float gy = 0.0f;
    float gradMag = 0.0f;
    uint8_t polarity = 0;
    int gradBin = 0;
};

struct ImageEdgeSamplingStats
{
    int rawEdgeCount = 0;
    int sampledCount = 0;
    int minTileCount = 0;
    int maxTileCount = 0;
    double avgTileCount = 0.0;
};

class ImageEdgeSampler
{
public:
    std::vector<ImageVotePoint> sample(const EdgeImageData& edgeData, const VotingConfig& config) const;
    std::vector<ImageVotePoint> sample(const EdgeImageData& edgeData,
                                       const VotingConfig& config,
                                       ImageEdgeSamplingStats* stats) const;

private:
    static int angleToBin(double angleRad, int binCount);
};

} // namespace ShapeMatch
