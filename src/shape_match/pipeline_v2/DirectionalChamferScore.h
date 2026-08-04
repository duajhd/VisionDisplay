#pragma once

#include <string>
#include <vector>

namespace ShapeMatch {

struct SegmentScore
{
    int segmentId = -1;

    int totalPoints = 0;
    int evaluatedPoints = 0;
    int matchedPoints = 0;

    double coverage = 0.0;
    double meanDistance = 0.0;
    double meanOrientationErrorDeg = 0.0;

    double distanceScore = 0.0;
    double orientationScore = 0.0;
    double segmentScore = 0.0;

    bool active = false;
};

struct DirectionalChamferScore
{
    double finalScore = 0.0;

    double distanceScore = 0.0;
    double orientationScore = 0.0;
    double coverageScore = 0.0;
    double segmentCoverageScore = 0.0;

    int totalPoints = 0;
    int evaluatedPoints = 0;
    int matchedPoints = 0;

    int totalSegments = 0;
    int activeSegments = 0;

    bool accepted = false;
    bool pruned = false;

    std::string rejectReason;

    std::vector<SegmentScore> segments;
};

} // namespace ShapeMatch
