#pragma once

namespace ShapeMatch {

struct VotingConfig
{
    bool enableOrientationVoting = true;

    int orientationBinCount = 72;
    float angleToleranceDeg = 7.5f;

    float accumulatorCellSizePx = 4.0f;
    int accumulatorNmsRadiusXY = 2;
    int accumulatorNmsRadiusTheta = 1;

    int maxTemplateVotePoints = 400;
    int maxImageVotePoints = 8000;

    int topVotePeaks = 200;
    int minVoteCount = 3;
    float minVoteScore = 0.0f;

    bool usePolarityInVoting = false;
    bool useGradientMagnitudeWeight = true;
    bool useTemplatePointWeight = true;

    int imageTileRows = 16;
    int imageTileCols = 16;
    int maxImagePointsPerTile = 40;

    bool enableVotingDebugReport = true;
};

} // namespace ShapeMatch
