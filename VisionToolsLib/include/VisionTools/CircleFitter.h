#pragma once

#include "VisionTools/CircleTypes.h"
#include "VisionTools/VisionTools_global.h"

#include <vector>

namespace VisionTools {

class VISIONTOOLS_API CircleFitter
{
public:
    CircleFitter();

    void setParams(const CircleFitParams& params);
    const CircleFitParams& params() const;

    CircleFitResult fit(const std::vector<EdgePoint>& points) const;

private:
    CircleModel fitAlgebraic(const std::vector<EdgePoint>& points) const;
    CircleModel fitRansac(const std::vector<EdgePoint>& points) const;
    CircleModel circleFromThreePoints(const EdgePoint& a, const EdgePoint& b, const EdgePoint& c) const;
    CircleModel optimizeGeometric(const CircleModel& initialCircle,
                                  const std::vector<EdgePoint>& points) const;
    double residual(const CircleModel& circle, const EdgePoint& point) const;
    void updateErrorMetrics(CircleFitResult* result) const;

    CircleFitParams m_params;
};

} // namespace VisionTools
