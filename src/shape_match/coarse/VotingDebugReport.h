#pragma once

#include "shape_match/coarse/SparseAccumulator.h"

#include <vector>

namespace ShapeMatch {

struct VotingDebugReport
{
    int templatePointCount = 0;
    int selectedTemplateVotePoints = 0;
    int rtEntryCount = 0;

    int rawImageEdgeCount = 0;
    int sampledImageVotePoints = 0;
    int minTileSampledCount = 0;
    int maxTileSampledCount = 0;
    double avgTileSampledCount = 0.0;

    int accumulatorNonZeroBins = 0;
    int totalVotes = 0;

    int peakCount = 0;
    std::vector<VotePeak> topPeaks;

    double buildRtTableMs = 0.0;
    double sampleImageEdgesMs = 0.0;
    double votingMs = 0.0;
    double peakFindMs = 0.0;
    double totalMs = 0.0;
};

} // namespace ShapeMatch
