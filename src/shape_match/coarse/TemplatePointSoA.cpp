#include "shape_match/coarse/TemplatePointSoA.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace ShapeMatch {

bool TemplatePointSoA::empty() const
{
    return x.empty();
}

int TemplatePointSoA::size() const
{
    return static_cast<int>(x.size());
}

float templatePointWeight(float weight, float gradMag)
{
    return std::max(0.01f, weight) * std::max(1.0f, gradMag / 255.0f);
}

TemplatePointSoA TemplatePointSoABuilder::build(const ShapeTemplateModel& model, bool enablePointOrdering) const
{
    TemplatePointSoA out;
    const int n = static_cast<int>(model.points.size());
    out.x.reserve(static_cast<size_t>(n));
    out.y.reserve(static_cast<size_t>(n));
    out.nx.reserve(static_cast<size_t>(n));
    out.ny.reserve(static_cast<size_t>(n));
    out.gx.reserve(static_cast<size_t>(n));
    out.gy.reserve(static_cast<size_t>(n));
    out.weight.reserve(static_cast<size_t>(n));
    out.gradMag.reserve(static_cast<size_t>(n));
    out.polarity.reserve(static_cast<size_t>(n));
    out.pointId.reserve(static_cast<size_t>(n));
    out.orderedIndices.resize(static_cast<size_t>(n));

    for (const TemplatePoint& pt : model.points) {
        const cv::Point2d g = norm(pt.gradientDir) > 1e-6 ? pt.gradientDir : pt.normal;
        out.x.push_back(static_cast<float>(pt.position.x));
        out.y.push_back(static_cast<float>(pt.position.y));
        out.nx.push_back(static_cast<float>(pt.normal.x));
        out.ny.push_back(static_cast<float>(pt.normal.y));
        out.gx.push_back(static_cast<float>(g.x));
        out.gy.push_back(static_cast<float>(g.y));
        out.weight.push_back(static_cast<float>(pt.weight));
        out.gradMag.push_back(static_cast<float>(pt.gradientMag));
        out.polarity.push_back(static_cast<uint8_t>(pt.polarity));
        out.pointId.push_back(pt.id);
        out.totalWeight += templatePointWeight(static_cast<float>(pt.weight), static_cast<float>(pt.gradientMag));
    }

    std::iota(out.orderedIndices.begin(), out.orderedIndices.end(), 0);
    if (enablePointOrdering) {
        std::stable_sort(out.orderedIndices.begin(), out.orderedIndices.end(), [&out](int lhs, int rhs) {
            const float lw = templatePointWeight(out.weight[static_cast<size_t>(lhs)], out.gradMag[static_cast<size_t>(lhs)]);
            const float rw = templatePointWeight(out.weight[static_cast<size_t>(rhs)], out.gradMag[static_cast<size_t>(rhs)]);
            if (std::abs(lw - rw) > 1e-6f) {
                return lw > rw;
            }
            return out.pointId[static_cast<size_t>(lhs)] < out.pointId[static_cast<size_t>(rhs)];
        });
    }
    if (out.totalWeight <= 0.0f) {
        out.totalWeight = static_cast<float>(std::max(1, n));
    }
    return out;
}

} // namespace ShapeMatch
