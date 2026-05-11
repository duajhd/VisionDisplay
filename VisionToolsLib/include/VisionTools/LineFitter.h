#pragma once

#include "VisionTools/LineTypes.h"
#include "VisionTools/VisionTools_global.h"

#include <vector>

namespace VisionTools {

class VISIONTOOLS_API LineFitter
{
public:
    LineFitter();

    void setParams(const LineFitParams& params);
    const LineFitParams& params() const;

    LineFitResult fit(const std::vector<EdgePoint>& points) const;

private:
    LineModel fitWeightedTls(const std::vector<EdgePoint>& points) const;
    LineModel fitWeightedTls(const std::vector<EdgePoint>& points,
                             const std::vector<double>& robustWeights) const;
    LineModel fitRansac(const std::vector<EdgePoint>& points) const;
    LineModel lineFromTwoPoints(const EdgePoint& a, const EdgePoint& b) const;
    LineModel robustRefit(const LineModel& initialLine, const std::vector<EdgePoint>& points) const;
    double residual(const LineModel& line, const EdgePoint& point) const;
    void updateErrorMetrics(LineFitResult* result) const;

    LineFitParams m_params;
};

} // namespace VisionTools
