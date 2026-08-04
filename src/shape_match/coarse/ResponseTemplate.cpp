#include "shape_match/coarse/ResponseTemplate.h"

#include "shape_match/core/ShapeMatchTypes.h"

#include <algorithm>
#include <cmath>
#include <map>

namespace ShapeMatch {

namespace {

int wrapBin(int bin, int count)
{
    if (count <= 0) {
        return 0;
    }
    bin %= count;
    return bin < 0 ? bin + count : bin;
}

int orientationBinFromVector(double x, double y, int binCount)
{
    const double angle = std::atan2(y, x);
    const double normalized = (angle + kPi) / (2.0 * kPi);
    int bin = static_cast<int>(std::floor(normalized * static_cast<double>(binCount)));
    return wrapBin(bin, binCount);
}

} // namespace

bool ResponseTemplate::empty() const
{
    return points.empty();
}

int ResponseTemplate::size() const
{
    return static_cast<int>(points.size());
}

ResponseTemplate ResponseTemplateBuilder::build(const ShapeTemplateModel& model,
                                                int level,
                                                const OrientationResponseConfig& config) const
{
    ResponseTemplate responseTemplate;
    responseTemplate.level = level;
    responseTemplate.orientationBinCount = std::max(1, config.orientationBinCount);
    if (model.points.empty()) {
        return responseTemplate;
    }

    const int maxPoints = std::max(1, config.maxTemplateResponsePoints);
    const int targetCount = std::min<int>(maxPoints, static_cast<int>(model.points.size()));
    const int chainCount = std::max(1, static_cast<int>(std::count_if(model.points.begin(), model.points.end(), [](const TemplatePoint&) {
                                         return true;
                                     })));
    (void)chainCount;

    std::map<int, std::vector<const TemplatePoint*>> byChain;
    for (const TemplatePoint& p : model.points) {
        byChain[p.chainId].push_back(&p);
    }

    std::vector<const TemplatePoint*> selected;
    selected.reserve(static_cast<size_t>(targetCount));
    const int perChain = std::max(1, targetCount / std::max(1, static_cast<int>(byChain.size())));
    for (auto& entry : byChain) {
        auto& points = entry.second;
        std::sort(points.begin(), points.end(), [](const TemplatePoint* a, const TemplatePoint* b) {
            const double wa = a->weight * std::max(1.0, a->gradientMag);
            const double wb = b->weight * std::max(1.0, b->gradientMag);
            return wa > wb;
        });
        const int keep = std::min<int>(perChain, static_cast<int>(points.size()));
        for (int i = 0; i < keep && static_cast<int>(selected.size()) < targetCount; ++i) {
            const int idx = keep <= 1 ? 0 : static_cast<int>(std::round(i * (points.size() - 1.0) / (keep - 1.0)));
            selected.push_back(points[static_cast<size_t>(std::clamp(idx, 0, static_cast<int>(points.size()) - 1))]);
        }
    }

    for (const TemplatePoint& p : model.points) {
        if (static_cast<int>(selected.size()) >= targetCount) {
            break;
        }
        const bool exists = std::find(selected.begin(), selected.end(), &p) != selected.end();
        if (!exists) {
            selected.push_back(&p);
        }
    }

    responseTemplate.points.reserve(selected.size());
    for (const TemplatePoint* p : selected) {
        cv::Point2d g = norm(p->gradientDir) > 1e-6 ? p->gradientDir : p->normal;
        g = normalized(g);
        ResponseTemplatePoint tp;
        tp.x = static_cast<float>(p->position.x);
        tp.y = static_cast<float>(p->position.y);
        tp.gx = static_cast<float>(g.x);
        tp.gy = static_cast<float>(g.y);
        tp.orientationBin = orientationBinFromVector(g.x, g.y, responseTemplate.orientationBinCount);
        const double gradientWeight = std::max(1.0, p->gradientMag) / 255.0;
        const double templateWeight = config.useTemplatePointWeight ? p->weight : 1.0;
        tp.weight = static_cast<float>(std::max(0.05, templateWeight * gradientWeight));
        tp.pointId = p->id;
        responseTemplate.points.push_back(tp);
    }
    return responseTemplate;
}

} // namespace ShapeMatch
