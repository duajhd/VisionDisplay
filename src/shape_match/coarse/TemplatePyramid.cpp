#include "shape_match/coarse/TemplatePyramid.h"

#include <algorithm>
#include <map>

namespace ShapeMatch {

namespace {

std::vector<TemplatePoint> uniformSampleByChain(const std::vector<TemplatePoint>& input, int maxPoints)
{
    if (maxPoints <= 0 || static_cast<int>(input.size()) <= maxPoints) {
        return input;
    }

    std::map<int, std::vector<const TemplatePoint*>> chains;
    for (const TemplatePoint& p : input) {
        chains[p.chainId].push_back(&p);
    }

    std::vector<TemplatePoint> sampled;
    sampled.reserve(static_cast<size_t>(maxPoints));
    const int chainCount = std::max(1, static_cast<int>(chains.size()));
    for (const auto& item : chains) {
        const std::vector<const TemplatePoint*>& chain = item.second;
        const int budget = std::max(1, maxPoints / chainCount);
        const int take = std::min<int>(budget, static_cast<int>(chain.size()));
        if (take >= static_cast<int>(chain.size())) {
            for (const TemplatePoint* p : chain) {
                sampled.push_back(*p);
            }
            continue;
        }
        for (int i = 0; i < take; ++i) {
            const double t = take == 1 ? 0.0 : static_cast<double>(i) / static_cast<double>(take - 1);
            const size_t index = std::min(chain.size() - 1,
                                          static_cast<size_t>(std::round(t * static_cast<double>(chain.size() - 1))));
            sampled.push_back(*chain[index]);
        }
    }

    if (static_cast<int>(sampled.size()) > maxPoints) {
        std::vector<TemplatePoint> trimmed;
        trimmed.reserve(static_cast<size_t>(maxPoints));
        for (int i = 0; i < maxPoints; ++i) {
            const double t = maxPoints == 1 ? 0.0 : static_cast<double>(i) / static_cast<double>(maxPoints - 1);
            const size_t index = std::min(sampled.size() - 1,
                                          static_cast<size_t>(std::round(t * static_cast<double>(sampled.size() - 1))));
            trimmed.push_back(sampled[index]);
        }
        return trimmed;
    }
    return sampled;
}

} // namespace

bool TemplatePyramid::build(const ShapeTemplateModel& baseModel, int levels, int maxPointsPerLevel)
{
    m_levels.clear();
    m_scales.clear();
    if (baseModel.empty() || levels <= 0) {
        return false;
    }

    levels = std::max(1, levels);
    maxPointsPerLevel = std::max(1, maxPointsPerLevel);
    for (int levelIndex = 0; levelIndex < levels; ++levelIndex) {
        const double scale = std::pow(0.5, levelIndex);
        ShapeTemplateModel model;
        model.templateId = baseModel.templateId;
        model.origin = baseModel.origin * scale;
        model.points = uniformSampleByChain(baseModel.points, maxPointsPerLevel);
        for (TemplatePoint& p : model.points) {
            p.position *= scale;
        }
        model.computeBoundingBox();
        m_levels.push_back(std::move(model));
        m_scales.push_back(scale);
    }
    return !m_levels.empty();
}

int TemplatePyramid::levelCount() const
{
    return static_cast<int>(m_levels.size());
}

const ShapeTemplateModel& TemplatePyramid::level(int levelIndex) const
{
    return m_levels.at(static_cast<size_t>(levelIndex));
}

double TemplatePyramid::scaleOfLevel(int levelIndex) const
{
    return m_scales.at(static_cast<size_t>(levelIndex));
}

} // namespace ShapeMatch
