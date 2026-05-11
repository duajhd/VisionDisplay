#pragma once

#include "VisionTools/EllipseTypes.h"
#include "VisionTools/VisionTools_global.h"

namespace VisionTools {

class VISIONTOOLS_API EllipseFitter
{
public:
    EllipseFitter();

    EllipseFitResult fit(const std::vector<EdgePoint>& points,
                         const EllipseModel& initial,
                         const EllipseFitOptions& options) const;

private:
    double residualApproxGeometric(const EdgePoint& point,
                                   const EllipseModel& ellipse) const;

    bool refineLevenbergMarquardt(const std::vector<EdgePoint>& points,
                                  EllipseModel& ellipse,
                                  const EllipseFitOptions& options,
                                  EllipseFitResult& result) const;

    bool normalizeEllipse(EllipseModel& ellipse, double minRadius) const;
    void updateErrorMetrics(EllipseFitResult* result) const;
};

} // namespace VisionTools
