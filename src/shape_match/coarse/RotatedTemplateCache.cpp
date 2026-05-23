#include "shape_match/coarse/RotatedTemplateCache.h"

#include "shape_match/core/ShapeMatchTypes.h"

#include <algorithm>
#include <cmath>

namespace ShapeMatch {

bool RotatedTemplateCache::build(const TemplatePointSoA& points, const std::vector<float>& thetaBinsRad)
{
    m_views.clear();
    if (points.empty() || thetaBinsRad.empty()) {
        return false;
    }
    m_views.reserve(thetaBinsRad.size());
    const size_t n = static_cast<size_t>(points.size());
    for (size_t bin = 0; bin < thetaBinsRad.size(); ++bin) {
        RotatedTemplateView view;
        view.thetaBin = static_cast<int>(bin);
        view.thetaRad = thetaBinsRad[bin];
        view.cosTheta = std::cos(view.thetaRad);
        view.sinTheta = std::sin(view.thetaRad);
        view.rx.resize(n);
        view.ry.resize(n);
        view.rgx.resize(n);
        view.rgy.resize(n);
        view.rnx.resize(n);
        view.rny.resize(n);
        for (size_t i = 0; i < n; ++i) {
            const float x = points.x[i];
            const float y = points.y[i];
            const float gx = points.gx[i];
            const float gy = points.gy[i];
            const float nx = points.nx[i];
            const float ny = points.ny[i];
            view.rx[i] = x * view.cosTheta - y * view.sinTheta;
            view.ry[i] = x * view.sinTheta + y * view.cosTheta;
            view.rgx[i] = gx * view.cosTheta - gy * view.sinTheta;
            view.rgy[i] = gx * view.sinTheta + gy * view.cosTheta;
            view.rnx[i] = nx * view.cosTheta - ny * view.sinTheta;
            view.rny[i] = nx * view.sinTheta + ny * view.cosTheta;
        }
        m_views.push_back(std::move(view));
    }
    return true;
}

const RotatedTemplateView* RotatedTemplateCache::findNearest(float thetaRad) const
{
    if (m_views.empty()) {
        return nullptr;
    }
    const RotatedTemplateView* best = &m_views.front();
    float bestAbs = std::abs(static_cast<float>(wrapToPi(thetaRad - best->thetaRad)));
    for (const RotatedTemplateView& view : m_views) {
        const float diff = std::abs(static_cast<float>(wrapToPi(thetaRad - view.thetaRad)));
        if (diff < bestAbs) {
            bestAbs = diff;
            best = &view;
        }
    }
    return best;
}

const RotatedTemplateView* RotatedTemplateCache::getByBin(int thetaBin) const
{
    if (thetaBin < 0 || thetaBin >= static_cast<int>(m_views.size())) {
        return nullptr;
    }
    return &m_views[static_cast<size_t>(thetaBin)];
}

int RotatedTemplateCache::binCount() const
{
    return static_cast<int>(m_views.size());
}

std::vector<float> buildUniformThetaBinsRad(double minDeg, double maxDeg, double stepDeg)
{
    std::vector<float> bins;
    const double step = std::max(0.1, stepDeg);
    for (double deg = minDeg; deg <= maxDeg + 1e-9; deg += step) {
        bins.push_back(static_cast<float>(degToRad(deg)));
    }
    if (bins.empty()) {
        bins.push_back(0.0f);
    }
    return bins;
}

} // namespace ShapeMatch
