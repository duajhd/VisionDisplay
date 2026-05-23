#pragma once

#include "VisionDisplay/OverlayData.h"
#include "VisionTools/Matching/ShapeTemplateModel.h"

struct ShapeModelOverlayOptions
{
    bool showAllEdges = false;
    bool showStablePoints = false;
    bool showRejectedPoints = true;
    bool showChains = true;
    bool showSampledPoints = true;
    bool showNormals = true;
    bool showTangents = false;
    bool showCurvature = true;
    bool showChainId = false;
    bool showPointId = false;
    int normalStep = 8;
    double normalLength = 12.0;
    int tangentStep = 8;
    double tangentLength = 10.0;
};

class ShapeModelOverlayBuilder
{
public:
    static VisionDisplay::VisionDisplayOverlayData build(
        const VisionTools::Matching::ShapeTemplateModel& model,
        int level,
        const ShapeModelOverlayOptions& options);
};
