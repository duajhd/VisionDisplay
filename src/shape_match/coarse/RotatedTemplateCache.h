#pragma once

#include "shape_match/coarse/TemplatePointSoA.h"

#include <vector>

namespace ShapeMatch {

struct RotatedTemplateView
{
    int thetaBin = -1;
    float thetaRad = 0.0f;
    float cosTheta = 1.0f;
    float sinTheta = 0.0f;
    std::vector<float> rx;
    std::vector<float> ry;
    std::vector<float> rgx;
    std::vector<float> rgy;
    std::vector<float> rnx;
    std::vector<float> rny;
};

class RotatedTemplateCache
{
public:
    bool build(const TemplatePointSoA& points, const std::vector<float>& thetaBinsRad);

    const RotatedTemplateView* findNearest(float thetaRad) const;
    const RotatedTemplateView* getByBin(int thetaBin) const;
    int binCount() const;

private:
    std::vector<RotatedTemplateView> m_views;
};

std::vector<float> buildUniformThetaBinsRad(double minDeg, double maxDeg, double stepDeg);

} // namespace ShapeMatch
