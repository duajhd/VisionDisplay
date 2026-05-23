#pragma once

#include <sstream>
#include <string>

namespace ShapeMatch {

struct CoarseMatchConfig
{
    struct RoiDistanceFieldConfig {
        bool enableRoiDistanceField = true;
        bool buildFullDistanceFieldForLevel0 = false;
        bool buildFullDistanceFieldForLevel1 = true;
        bool buildFullDistanceFieldForCoarseLevels = true;
        int largeImageMinPixelsForRoiDistanceField = 8000000;
        int roiPaddingPxLevel0 = 64;
        int roiPaddingPxLevel1 = 32;
        int maxRoiDistanceFields = 512;
        int maxTotalRoiPixelsForDistanceField = 12000000;
        int minRoiWidth = 128;
        int minRoiHeight = 128;
        int maxRoiWidth = 2048;
        int maxRoiHeight = 2048;
        bool mergeOverlappingRois = true;
        double roiMergeOverlapRatio = 0.30;
        bool enableLazyRoiDistanceFieldBuild = true;
        bool enableRoiDistanceFieldCache = true;
        bool fallbackToLocalWindowIfNoRoiField = true;
        bool allowFullLevel0DistanceFieldFallback = false;
    };

    struct Level0RefineBudgetConfig {
        bool enableLevel0BudgetV2 = true;
        int maxLevel0ParentCandidates = 180;
        int maxLevel0ParentPerGridCell = 5;
        int level0ParentGridRows = 4;
        int level0ParentGridCols = 4;
        int maxLevel0ChildrenPerParent = 240;
        int maxLevel0RawChildren = 120000;
        int maxLevel0ScoredChildren = 24000;
        int targetLevel0ScoredChildren = 16000;
        int maxLevel0ChildrenPerRegion = 800;
        int maxLevel0ParentsPerRegion = 8;
        bool enableRegionAwareBudget = true;
        bool enableScoreGapParentPruning = true;
        double parentScoreGapKeep = 0.20;
        bool enableLevel0AdaptiveStep = true;
        int level0MinTranslationStepPx = 1;
        int level0MaxTranslationStepPx = 3;
        double level0MinAngleStepDeg = 1.0;
        double level0MaxAngleStepDeg = 2.0;
    };

    enum class CoarseSearchMode {
        Recall,
        Balanced,
        Fast
    };

    enum class CoarsePreset {
        Fast,
        Balanced,
        Recall
    };

    CoarseSearchMode searchMode = CoarseSearchMode::Balanced;
    CoarsePreset currentPreset = CoarsePreset::Balanced;
    int pyramidLevels = 4;

    int topKPerLevel = 50;
    int finalTopK = 20;
    int beamWidth = 50;

    double minAngleDeg = -180.0;
    double maxAngleDeg = 180.0;

    double coarseAngleStepDeg = 10.0;
    double fineAngleStepDeg = 2.0;

    double minScale = 1.0;
    double maxScale = 1.0;
    double scaleStep = 0.05;
    bool enableScaleSearch = false;

    int coarseTranslationStepPx = 8;
    int fineTranslationStepPx = 1;

    int localRefineRadiusPx = 8;
    int localRefineAngleRadiusDeg = 6;

    int maxTemplatePointsPerLevel = 800;
    int minTemplatePointsForScore = 30;

    double fastDistanceSigma = 3.0;
    double fastAngleSigmaDeg = 25.0;

    double earlyExitMinPossibleScore = 0.10;
    double earlyExitTopKMargin = 0.05;

    double minFastScoreForPropagation = 0.05;

    bool useGradientOrientation = true;
    bool usePolarity = true;
    bool useDistanceMap = true;
    bool enableEarlyExit = true;
    bool enableCoarseOverlay = true;
    bool enableVerboseLevelLog = true;
    bool enableProfiler = true;

#ifndef NDEBUG
    bool enableStageTrace = true;
    bool enableMissedTargetStageTrace = true;
#else
    bool enableStageTrace = false;
    bool enableMissedTargetStageTrace = false;
#endif
    int maxTraceCandidatesPerStage = 500;
    int maxTraceCandidatesNearTarget = 100;
    double traceNearDxyThresholdPx = 15.0;
    double traceNearAngleThresholdDeg = 15.0;
    bool traceSuppressedCandidates = true;
    bool traceBudgetClippedCandidates = true;
    bool tracePrunedCandidates = true;
    bool traceRoiVoteStats = true;

    int maxCandidatesEvaluatedPerLevel = 200000;

    bool enableOrientationVoting = true;
    bool fallbackToGridSearchIfVotingTooFew = true;
    int minVotingCandidates = 20;
    int maxVotingCandidatesToScore = 500;
    bool enableVotingOverlay = true;
    bool enableVotingReport = true;

    bool enableSearchRoi = false;
    int searchRoiX = 0;
    int searchRoiY = 0;
    int searchRoiWidth = 0;
    int searchRoiHeight = 0;

    bool enableSpatialDiversity = true;
    int diversityGridRows = 4;
    int diversityGridCols = 4;
    int topKPerGridCell = 5;
    int globalTopKAfterDiversity = 80;

    double nmsTranslationThresholdPx = 6.0;
    double nmsAngleThresholdDeg = 5.0;
    double nmsScaleThreshold = 0.03;

    bool enableDistanceFieldScoring = true;
    bool enableNearestEdgeField = true;
    bool enableOrientationField = true;
    bool enableLocalSearchFallback = true;
    bool enableGreedyUpperBoundPruning = true;
    bool enablePointOrdering = true;
    bool enableScorerCompareTest = true;
    float maxDistanceForScore = 10.0f;
    float distanceScoreSigma = 3.0f;
    float orientationScoreSigmaDeg = 25.0f;
    float fastWDistance = 0.60f;
    float fastWOrientation = 0.35f;
    float fastWPolarity = 0.05f;
    int upperBoundCheckInterval = 32;
    float upperBoundMargin = 0.02f;
    int minEvaluatedPointsBeforePruning = 32;
    float minFastScoreForKeep = 0.0f;

    bool enableCandidateBudgetPolicy = true;
    bool enableAdaptiveCandidateBudget = true;
    int maxCandidatesPerLevel = 30000;
    int maxCandidatesLevel0 = 12000;
    int maxCandidatesLocalRefine = 12000;
    int maxChildrenPerParentLevel0 = 420;
    int maxChildrenPerParentCoarse = 240;
    int maxBeamForLocalRefineLevel0 = 30;
    int maxBeamForLocalRefineCoarse = 50;
    bool enableAdaptiveRefineWindow = true;
    bool enableScoreBasedBeamReduction = true;
    bool enableParentDiversityBeforeRefine = true;
    double highConfidenceScore = 0.65;
    double mediumConfidenceScore = 0.45;
    int level0HighConfRadiusPx = 4;
    int level0MediumConfRadiusPx = 6;
    int level0LowConfRadiusPx = 8;
    int coarseHighConfRadiusPx = 8;
    int coarseMediumConfRadiusPx = 8;
    int coarseLowConfRadiusPx = 12;
    double level0HighConfAngleRadiusDeg = 4.0;
    double level0MediumConfAngleRadiusDeg = 6.0;
    double level0LowConfAngleRadiusDeg = 6.0;
    int parentDiversityGridRows = 4;
    int parentDiversityGridCols = 4;
    int maxParentsPerGridCell = 3;
    int maxParentsForLevel0Refine = 30;

    bool enableTemplatePointSoA = true;
    bool enableRotatedTemplateCache = true;
    double rotatedCacheAngleStepDeg = 1.0;
    bool allowDynamicRotatedCache = true;

    bool enableParallelCandidateScoring = true;
    int numScoringThreads = 0;
    int minCandidatesForParallelScoring = 1000;
    RoiDistanceFieldConfig roiDistanceField;
    Level0RefineBudgetConfig level0BudgetV2;

    static const char* presetName(CoarsePreset preset)
    {
        switch (preset) {
        case CoarsePreset::Fast:
            return "Fast";
        case CoarsePreset::Balanced:
            return "Balanced";
        case CoarsePreset::Recall:
            return "Recall";
        }
        return "Balanced";
    }

    void applyPreset(CoarsePreset preset)
    {
        currentPreset = preset;

        enableCandidateBudgetPolicy = true;
        enableAdaptiveCandidateBudget = true;
        enableAdaptiveRefineWindow = true;
        enableScoreBasedBeamReduction = true;
        enableParentDiversityBeforeRefine = true;
        enableTemplatePointSoA = true;
        enableRotatedTemplateCache = true;
        enableParallelCandidateScoring = true;
        enableGreedyUpperBoundPruning = true;
        enableSpatialDiversity = true;
        fastWPolarity = 0.05f;

        if (preset == CoarsePreset::Fast) {
            maxCandidatesLevel0 = 12000;
            maxCandidatesLocalRefine = 12000;
            maxCandidatesPerLevel = 30000;
            maxChildrenPerParentLevel0 = 420;
            maxChildrenPerParentCoarse = 240;
            maxBeamForLocalRefineLevel0 = 30;
            maxBeamForLocalRefineCoarse = 50;
            maxParentsForLevel0Refine = 30;
            maxParentsPerGridCell = 3;
            level0HighConfRadiusPx = 4;
            level0MediumConfRadiusPx = 6;
            level0LowConfRadiusPx = 8;
            level0HighConfAngleRadiusDeg = 4.0;
            level0MediumConfAngleRadiusDeg = 6.0;
            level0LowConfAngleRadiusDeg = 6.0;
            upperBoundMargin = 0.02f;
            minEvaluatedPointsBeforePruning = 32;
            minCandidatesForParallelScoring = 1000;
            return;
        }

        if (preset == CoarsePreset::Balanced) {
            maxCandidatesLevel0 = 24000;
            maxCandidatesLocalRefine = 24000;
            maxCandidatesPerLevel = 45000;
            maxChildrenPerParentLevel0 = 700;
            maxChildrenPerParentCoarse = 320;
            maxBeamForLocalRefineLevel0 = 50;
            maxBeamForLocalRefineCoarse = 60;
            maxParentsForLevel0Refine = 50;
            maxParentsPerGridCell = 5;
            level0HighConfRadiusPx = 3;
            level0MediumConfRadiusPx = 6;
            level0LowConfRadiusPx = 10;
            level0HighConfAngleRadiusDeg = 3.0;
            level0MediumConfAngleRadiusDeg = 5.0;
            level0LowConfAngleRadiusDeg = 8.0;
            upperBoundMargin = 0.005f;
            minEvaluatedPointsBeforePruning = 32;
            minCandidatesForParallelScoring = 1000;
            return;
        }

        maxCandidatesLevel0 = 36000;
        maxCandidatesLocalRefine = 36000;
        maxCandidatesPerLevel = 60000;
        maxChildrenPerParentLevel0 = 1100;
        maxChildrenPerParentCoarse = 420;
        maxBeamForLocalRefineLevel0 = 80;
        maxBeamForLocalRefineCoarse = 80;
        maxParentsForLevel0Refine = 80;
        maxParentsPerGridCell = 6;
        level0HighConfRadiusPx = 4;
        level0MediumConfRadiusPx = 8;
        level0LowConfRadiusPx = 12;
        level0HighConfAngleRadiusDeg = 4.0;
        level0MediumConfAngleRadiusDeg = 7.0;
        level0LowConfAngleRadiusDeg = 10.0;
        upperBoundMargin = 0.0f;
        minEvaluatedPointsBeforePruning = 48;
        minCandidatesForParallelScoring = 1000;
    }

    static CoarseMatchConfig makeFastPreset()
    {
        CoarseMatchConfig config;
        config.applyPreset(CoarsePreset::Fast);
        return config;
    }

    static CoarseMatchConfig makeBalancedPreset()
    {
        CoarseMatchConfig config;
        config.applyPreset(CoarsePreset::Balanced);
        return config;
    }

    static CoarseMatchConfig makeRecallPreset()
    {
        CoarseMatchConfig config;
        config.applyPreset(CoarsePreset::Recall);
        return config;
    }

    std::string toString() const
    {
        std::ostringstream out;
        out << "currentPreset=" << presetName(currentPreset)
            << " pyramidLevels=" << pyramidLevels
            << " topKPerLevel=" << topKPerLevel
            << " finalTopK=" << finalTopK
            << " beamWidth=" << beamWidth
            << " angleRange=[" << minAngleDeg << ',' << maxAngleDeg << ']'
            << " coarseAngleStepDeg=" << coarseAngleStepDeg
            << " fineAngleStepDeg=" << fineAngleStepDeg
            << " scaleRange=[" << minScale << ',' << maxScale << ']'
            << " scaleStep=" << scaleStep
            << " enableScaleSearch=" << (enableScaleSearch ? "true" : "false")
            << " coarseTranslationStepPx=" << coarseTranslationStepPx
            << " fineTranslationStepPx=" << fineTranslationStepPx
            << " localRefineRadiusPx=" << localRefineRadiusPx
            << " localRefineAngleRadiusDeg=" << localRefineAngleRadiusDeg
            << " maxTemplatePointsPerLevel=" << maxTemplatePointsPerLevel
            << " minTemplatePointsForScore=" << minTemplatePointsForScore
            << " fastDistanceSigma=" << fastDistanceSigma
            << " fastAngleSigmaDeg=" << fastAngleSigmaDeg
            << " enableStageTrace=" << (enableStageTrace ? "true" : "false")
            << " enableMissedTargetStageTrace=" << (enableMissedTargetStageTrace ? "true" : "false")
            << " maxTraceCandidatesPerStage=" << maxTraceCandidatesPerStage
            << " maxTraceCandidatesNearTarget=" << maxTraceCandidatesNearTarget
            << " traceNearDxyThresholdPx=" << traceNearDxyThresholdPx
            << " traceNearAngleThresholdDeg=" << traceNearAngleThresholdDeg
            << " maxCandidatesEvaluatedPerLevel=" << maxCandidatesEvaluatedPerLevel
            << " enableOrientationVoting=" << (enableOrientationVoting ? "true" : "false")
            << " fallbackToGridSearchIfVotingTooFew=" << (fallbackToGridSearchIfVotingTooFew ? "true" : "false")
            << " minVotingCandidates=" << minVotingCandidates
            << " maxVotingCandidatesToScore=" << maxVotingCandidatesToScore
            << " enableVotingOverlay=" << (enableVotingOverlay ? "true" : "false")
            << " enableVotingReport=" << (enableVotingReport ? "true" : "false")
            << " enableSearchRoi=" << (enableSearchRoi ? "true" : "false")
            << " searchRoi=[" << searchRoiX << ',' << searchRoiY << ','
            << searchRoiWidth << ',' << searchRoiHeight << ']'
            << " enableSpatialDiversity=" << (enableSpatialDiversity ? "true" : "false")
            << " diversityGrid=" << diversityGridRows << "x" << diversityGridCols
            << " topKPerGridCell=" << topKPerGridCell
            << " enableDistanceFieldScoring=" << (enableDistanceFieldScoring ? "true" : "false")
            << " enableNearestEdgeField=" << (enableNearestEdgeField ? "true" : "false")
            << " enableOrientationField=" << (enableOrientationField ? "true" : "false")
            << " enableLocalSearchFallback=" << (enableLocalSearchFallback ? "true" : "false")
            << " enableGreedyUpperBoundPruning=" << (enableGreedyUpperBoundPruning ? "true" : "false")
            << " enablePointOrdering=" << (enablePointOrdering ? "true" : "false")
            << " maxDistanceForScore=" << maxDistanceForScore
            << " distanceScoreSigma=" << distanceScoreSigma
            << " orientationScoreSigmaDeg=" << orientationScoreSigmaDeg
            << " upperBoundCheckInterval=" << upperBoundCheckInterval
            << " upperBoundMargin=" << upperBoundMargin
            << " minEvaluatedPointsBeforePruning=" << minEvaluatedPointsBeforePruning
            << " minFastScoreForKeep=" << minFastScoreForKeep
            << " enableCandidateBudgetPolicy=" << (enableCandidateBudgetPolicy ? "true" : "false")
            << " enableAdaptiveCandidateBudget=" << (enableAdaptiveCandidateBudget ? "true" : "false")
            << " maxCandidatesPerLevel=" << maxCandidatesPerLevel
            << " maxCandidatesLevel0=" << maxCandidatesLevel0
            << " maxCandidatesLocalRefine=" << maxCandidatesLocalRefine
            << " maxChildrenPerParentLevel0=" << maxChildrenPerParentLevel0
            << " maxChildrenPerParentCoarse=" << maxChildrenPerParentCoarse
            << " maxBeamForLocalRefineLevel0=" << maxBeamForLocalRefineLevel0
            << " maxBeamForLocalRefineCoarse=" << maxBeamForLocalRefineCoarse
            << " enableAdaptiveRefineWindow=" << (enableAdaptiveRefineWindow ? "true" : "false")
            << " enableParentDiversityBeforeRefine=" << (enableParentDiversityBeforeRefine ? "true" : "false")
            << " enableTemplatePointSoA=" << (enableTemplatePointSoA ? "true" : "false")
            << " enableRotatedTemplateCache=" << (enableRotatedTemplateCache ? "true" : "false")
            << " rotatedCacheAngleStepDeg=" << rotatedCacheAngleStepDeg
            << " enableParallelCandidateScoring=" << (enableParallelCandidateScoring ? "true" : "false")
            << " numScoringThreads=" << numScoringThreads
            << " minCandidatesForParallelScoring=" << minCandidatesForParallelScoring
            << " enableRoiDistanceField=" << (roiDistanceField.enableRoiDistanceField ? "true" : "false")
            << " buildFullDistanceFieldForLevel0=" << (roiDistanceField.buildFullDistanceFieldForLevel0 ? "true" : "false")
            << " buildFullDistanceFieldForLevel1=" << (roiDistanceField.buildFullDistanceFieldForLevel1 ? "true" : "false")
            << " maxRoiDistanceFields=" << roiDistanceField.maxRoiDistanceFields
            << " maxTotalRoiPixelsForDistanceField=" << roiDistanceField.maxTotalRoiPixelsForDistanceField
            << " enableLevel0BudgetV2=" << (level0BudgetV2.enableLevel0BudgetV2 ? "true" : "false")
            << " maxLevel0ParentCandidates=" << level0BudgetV2.maxLevel0ParentCandidates
            << " maxLevel0ChildrenPerParent=" << level0BudgetV2.maxLevel0ChildrenPerParent
            << " maxLevel0RawChildren=" << level0BudgetV2.maxLevel0RawChildren
            << " maxLevel0ScoredChildren=" << level0BudgetV2.maxLevel0ScoredChildren;
        return out.str();
    }
};

} // namespace ShapeMatch
