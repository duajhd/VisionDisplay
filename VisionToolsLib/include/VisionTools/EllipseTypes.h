#pragma once

#include "VisionTools/CaliperTypes.h"
#include "VisionTools/EdgePoint.h"
#include "VisionTools/VisionTools_global.h"

#include <QString>

#include <vector>

namespace VisionTools {

struct VISIONTOOLS_API EllipseModel
{
    double centerX = 0.0;
    double centerY = 0.0;
    double radiusA = 0.0;
    double radiusB = 0.0;
    double angleDeg = 0.0;
};

enum class EllipseSearchDirection
{
    FromOutsideToInside,
    FromInsideToOutside
};

enum class RobustLossType
{
    None,
    Huber,
    Tukey
};

struct VISIONTOOLS_API EllipseRobustOptions
{
    RobustLossType lossType = RobustLossType::Huber;
    double huberDelta = 2.0;
    double tukeyC = 4.685;
    int irlsIterations = 10;
    double minWeight = 1.0e-6;
};

struct VISIONTOOLS_API EllipseFitOptions
{
    double maxResidual = 2.0;
    int minInlierCount = 5;
    int maxIterations = 30;
    double initialLambda = 1.0e-3;
    double minRadius = 1.0;
    EllipseRobustOptions robust;
};

struct VISIONTOOLS_API EllipseFitResult
{
    bool ok = false;
    EllipseModel ellipse;
    std::vector<EdgePoint> inputPoints;
    std::vector<EdgePoint> inlierPoints;
    std::vector<EdgePoint> outlierPoints;
    std::vector<double> pointResiduals;
    std::vector<double> pointWeights;
    double rmsError = 0.0;
    double maxError = 0.0;
    double score = 0.0;
    QString message;
};

struct VISIONTOOLS_API EllipseFitDiagnostics
{
    int inputCount = 0;
    int candidateCount = 0;
    int inlierCount = 0;
    int outlierCount = 0;

    double inlierRatio = 0.0;

    double rmsError = 0.0;
    double maxError = 0.0;
    double medianError = 0.0;
    double meanAbsError = 0.0;

    double centerShiftFromExpected = 0.0;
    double radiusAErrorRatio = 0.0;
    double radiusBErrorRatio = 0.0;
    double angleErrorDeg = 0.0;

    double conditionHint = 0.0;

    QString failureReason;
};

struct VISIONTOOLS_API FindEllipseParams
{
    double centerX = 0.0;
    double centerY = 0.0;
    double radiusA = 100.0;
    double radiusB = 60.0;
    double angleDeg = 0.0;

    double startAngleDeg = 0.0;
    double spanAngleDeg = 360.0;

    int caliperCount = 36;
    double searchLength = 40.0;
    double projectionWidth = 8.0;

    EdgePolarity polarity = EdgePolarity::Any;
    EdgeSelection selection = EdgeSelection::Strongest;

    double minResponse = 5.0;
    double maxResidual = 2.0;
    int maxIterations = 30;
    EllipseRobustOptions robust;

    EllipseSearchDirection searchDirection = EllipseSearchDirection::FromOutsideToInside;

    bool useExpectedAsInitial = true;
    bool enableRobustFiltering = true;
    bool enableNonlinearRefine = true;
};

struct VISIONTOOLS_API FindEllipseResult
{
    bool ok = false;

    EllipseModel expectedEllipse;
    EllipseModel fittedEllipse;

    std::vector<CaliperRegion> calipers;
    std::vector<EdgePoint> caliperHitPoints;
    std::vector<EdgePoint> inputPoints;
    std::vector<EdgePoint> inlierPoints;
    std::vector<EdgePoint> outlierPoints;
    std::vector<double> pointResiduals;
    std::vector<double> pointWeights;

    double rmsError = 0.0;
    double maxError = 0.0;
    double score = 0.0;

    EllipseFitDiagnostics diagnostics;

    QString message;
};

} // namespace VisionTools
