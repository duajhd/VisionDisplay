#pragma once

namespace ShapeMatch {

struct OrientationResponseConfig
{
    bool enableOrientationResponse = true;

    int orientationBinCount = 16;
    int thetaBinCount = 72;
    double thetaMinDeg = -180.0;
    double thetaMaxDeg = 180.0;

    bool enableOrientationSpreading = true;
    int orientationSpreadRadiusBins = 1;

    bool enableSpatialSpreading = true;
    int spatialSpreadRadiusPx = 4;

    int maxTemplateResponsePoints = 400;
    int minTemplateResponsePoints = 50;

    bool useGradientMagnitudeWeight = true;
    bool useTemplatePointWeight = true;
    bool usePolarityInResponse = false;

    bool useTilePeakQuota = true;
    int tileRows = 8;
    int tileCols = 8;
    int maxPeaksPerTile = 5;

    int maxPeaksPerTheta = 200;
    int maxTotalPeaks = 1000;

    double minResponseScore = 0.05;

    int nmsRadiusPx = 8;
    int nmsThetaRadiusBins = 1;

    bool exportResponseDebugImages = true;
    bool exportResponseCsv = true;
    bool exportResponseJson = true;
    bool exportResponseSummary = true;

    bool enableResponseProfiler = true;
};

} // namespace ShapeMatch
