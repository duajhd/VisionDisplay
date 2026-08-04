#include "shape_match/pipeline_v2/SegmentTemplate.h"

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

int binFromVector(double x, double y, int count)
{
    const double angle = std::atan2(y, x);
    return wrapBin(static_cast<int>(std::floor((angle + kPi) / (2.0 * kPi) * count)), count);
}

} // namespace

bool SegmentTemplate::empty() const
{
    return points.empty() || segments.empty();
}

int SegmentTemplate::pointCount() const
{
    return static_cast<int>(points.size());
}

int SegmentTemplate::segmentCount() const
{
    return static_cast<int>(segments.size());
}

SegmentTemplate SegmentTemplateBuilder::build(const ShapeTemplateModel& model,
                                              int level,
                                              const DirectionalChamferConfig& config) const
{
    SegmentTemplate templ;
    templ.level = level;
    templ.orientationBinCount = std::max(1, config.orientationBinCount);
    if (model.points.empty()) {
        return templ;
    }

    std::map<int, std::vector<const TemplatePoint*>> byChain;
    for (const TemplatePoint& p : model.points) {
        byChain[p.chainId].push_back(&p);
    }

    const int maxPoints = std::max(1, config.maxTemplateVerifyPoints);
    const int maxSegments = std::max(1, config.maxSegments);
    const int chainLimit = std::min<int>(maxSegments, static_cast<int>(byChain.size()));
    const int perSegmentTarget = std::max(config.minPointsPerSegment, maxPoints / std::max(1, chainLimit));

    for (auto& entry : byChain) {
        if (static_cast<int>(templ.segments.size()) >= maxSegments ||
            static_cast<int>(templ.points.size()) >= maxPoints) {
            break;
        }
        const std::vector<const TemplatePoint*>& chain = entry.second;
        if (static_cast<int>(chain.size()) < config.minPointsPerSegment) {
            continue;
        }
        const int start = static_cast<int>(templ.points.size());
        const int keep = std::min<int>(perSegmentTarget, std::min<int>(static_cast<int>(chain.size()), maxPoints - start));
        float totalWeight = 0.0f;
        for (int i = 0; i < keep; ++i) {
            const int idx = keep <= 1 ? 0 : static_cast<int>(std::round(i * (chain.size() - 1.0) / (keep - 1.0)));
            const TemplatePoint* p = chain[static_cast<size_t>(std::clamp(idx, 0, static_cast<int>(chain.size()) - 1))];
            cv::Point2d g = norm(p->gradientDir) > 1e-6 ? p->gradientDir : p->normal;
            g = normalized(g);
            SegmentVerifyPoint vp;
            vp.x = static_cast<float>(p->position.x);
            vp.y = static_cast<float>(p->position.y);
            vp.gx = static_cast<float>(g.x);
            vp.gy = static_cast<float>(g.y);
            vp.weight = static_cast<float>(std::max(0.05, p->weight * std::max(1.0, p->gradientMag) / 255.0));
            vp.orientationBin = binFromVector(g.x, g.y, templ.orientationBinCount);
            vp.pointId = p->id;
            vp.segmentId = entry.first;
            totalWeight += vp.weight;
            templ.points.push_back(vp);
        }
        const int count = static_cast<int>(templ.points.size()) - start;
        if (count >= config.minPointsPerSegment) {
            templ.segments.push_back({entry.first, start, count, totalWeight});
        } else {
            templ.points.resize(static_cast<size_t>(start));
        }
    }
    return templ;
}

} // namespace ShapeMatch
