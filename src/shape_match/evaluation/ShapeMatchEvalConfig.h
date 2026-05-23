#pragma once

namespace ShapeMatch {

struct ShapeMatchEvalConfig
{
    int topK = 10;
    double searchRadiusPx = 5.0;
    int localSearchRadiusPx = 5;
    double coarseDistanceSigma = 3.0;
    double refineDistanceSigma = 1.0;
    double angleSigmaDeg = 20.0;
    double inlierDistanceThresholdPx = 3.0;
    double inlierAngleThresholdDeg = 25.0;
    int minValidPoints = 30;
    double minCoverageRatio = 0.20;
    double minInlierRatio = 0.15;
    double maxMedianErrorPx = 5.0;
    double maxP90ErrorPx = 10.0;
    double wCoverage = 0.30;
    double wDistance = 0.30;
    double wOrientation = 0.20;
    double wPolarity = 0.10;
    double wInlier = 0.10;
    double wPointDistance = 0.50;
    double wPointOrientation = 0.30;
    double wPointPolarity = 0.20;
    double multiTargetAngleCostWeight = 0.50;
    double xyOkThresholdPx = 5.0;
    double angleOkThresholdDeg = 5.0;
    double scaleOkThreshold = 0.02;
    bool enablePolarityCheck = true;
    bool enableOrientationCheck = true;
    bool enableDistanceMap = true;
    bool enableDistanceFieldEvaluation = true;
    bool enableNearestEdgeFieldEvaluation = true;
    bool enableFinalRankerCandidateNms = true;
    bool enableParallelFinalRanking = true;
    bool enablePointEvaluations = false;
    bool enablePointEvaluationsForTopKOnly = true;
    int pointEvaluationTopK = 3;
    int maxFinalRankerInputCandidates = 160;
    int targetFinalRankerInputCandidates = 160;
    double finalNmsTranslationPx = 5.0;
    double finalNmsAngleDeg = 2.0;
    double finalNmsScale = 0.02;
    bool allowLocalSearchFallbackInEvaluator = true;
    int evaluatorLocalSearchRadiusPx = 5;
    int finalRankerNumThreads = 0;
    bool exportPointEvalCsv = true;
    bool exportJsonReport = true;
    bool exportCandidateCsv = true;
};

} // namespace ShapeMatch
