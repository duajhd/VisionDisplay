#pragma once

#include "VisionTools/VisionTools_global.h"

#include <QRect>
#include <QString>

#include <vector>

namespace VisionTools::Matching {

struct VISIONTOOLS_API GrayMatchCandidate
{
    double x = 0.0;
    double y = 0.0;
    double score = 0.0;
};

struct VISIONTOOLS_API GrayMatchParams
{
    QRect searchRoi;
    double minScore = 0.7;
    int pyramidLevels = 1;
    int stepX = 1;
    int stepY = 1;
    int topK = 5;
    bool enableSubpixelRefine = true;
};

struct VISIONTOOLS_API GrayMatchResult
{
    bool ok = false;
    double x = 0.0;
    double y = 0.0;
    double score = 0.0;
    std::vector<GrayMatchCandidate> candidates;
    QString message;
};

} // namespace VisionTools::Matching
