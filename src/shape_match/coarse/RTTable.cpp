#include "shape_match/coarse/RTTable.h"

#include <algorithm>
#include <cmath>
#include <map>

namespace ShapeMatch {

namespace {

cv::Point2d gradientDirection(const TemplatePoint& point)
{
    if (norm(point.gradientDir) > 1e-6) {
        return normalized(point.gradientDir);
    }
    if (norm(point.normal) > 1e-6) {
        return normalized(point.normal);
    }
    return cv::Point2d(0.0, 0.0);
}

} // namespace

bool RTTable::build(const ShapeTemplateModel& model, const VotingConfig& config)
{
    m_config = config;
    const int binCount = std::max(1, config.orientationBinCount);
    m_entriesByImageGradBin.assign(static_cast<size_t>(binCount), {});
    m_totalEntryCount = 0;
    m_selectedTemplatePointCount = 0;

    if (model.empty()) {
        return false;
    }

    const std::vector<const TemplatePoint*> selected = selectTemplatePoints(model, config);
    m_selectedTemplatePointCount = static_cast<int>(selected.size());
    if (selected.empty()) {
        return false;
    }

    const int toleranceBins = std::max(0, static_cast<int>(std::ceil(
        static_cast<double>(config.angleToleranceDeg) / (360.0 / static_cast<double>(binCount)))));

    for (int thetaBin = 0; thetaBin < binCount; ++thetaBin) {
        const double theta = 2.0 * kPi * static_cast<double>(thetaBin) / static_cast<double>(binCount);
        const double c = std::cos(theta);
        const double s = std::sin(theta);
        for (const TemplatePoint* point : selected) {
            const cv::Point2d grad = gradientDirection(*point);
            if (norm(grad) <= 1e-6) {
                continue;
            }
            const double dx = c * point->position.x - s * point->position.y;
            const double dy = s * point->position.x + c * point->position.y;
            const double ngx = c * grad.x - s * grad.y;
            const double ngy = s * grad.x + c * grad.y;
            const int imageGradBin = angleToBin(std::atan2(ngy, ngx), binCount);
            const int templateGradBin = angleToBin(std::atan2(grad.y, grad.x), binCount);

            RTEntry entry;
            entry.dx = static_cast<float>(dx);
            entry.dy = static_cast<float>(dy);
            entry.nx = static_cast<float>(ngx);
            entry.ny = static_cast<float>(ngy);
            entry.weight = static_cast<float>(config.useTemplatePointWeight ? point->weight : 1.0);
            entry.polarity = polarityToByte(point->polarity);
            entry.pointId = point->id;
            entry.thetaBin = thetaBin;
            entry.templateGradBin = templateGradBin;

            for (int offset = -toleranceBins; offset <= toleranceBins; ++offset) {
                int bin = (imageGradBin + offset) % binCount;
                if (bin < 0) {
                    bin += binCount;
                }
                m_entriesByImageGradBin[static_cast<size_t>(bin)].push_back(entry);
                ++m_totalEntryCount;
            }
        }
    }
    return m_totalEntryCount > 0;
}

const std::vector<RTEntry>& RTTable::entriesForImageGradientBin(int imageGradBin) const
{
    static const std::vector<RTEntry> empty;
    if (m_entriesByImageGradBin.empty()) {
        return empty;
    }
    const int binCount = static_cast<int>(m_entriesByImageGradBin.size());
    imageGradBin %= binCount;
    if (imageGradBin < 0) {
        imageGradBin += binCount;
    }
    return m_entriesByImageGradBin[static_cast<size_t>(imageGradBin)];
}

int RTTable::orientationBinCount() const
{
    return static_cast<int>(m_entriesByImageGradBin.size());
}

int RTTable::totalEntryCount() const
{
    return m_totalEntryCount;
}

int RTTable::selectedTemplatePointCount() const
{
    return m_selectedTemplatePointCount;
}

int RTTable::angleToBin(double angleRad, int binCount)
{
    angleRad = wrapToPi(angleRad);
    if (angleRad < 0.0) {
        angleRad += 2.0 * kPi;
    }
    int bin = static_cast<int>(std::floor(angleRad * static_cast<double>(binCount) / (2.0 * kPi)));
    return std::clamp(bin, 0, binCount - 1);
}

uint8_t RTTable::polarityToByte(EdgePolarity polarity)
{
    return static_cast<uint8_t>(polarity);
}

std::vector<const TemplatePoint*> RTTable::selectTemplatePoints(const ShapeTemplateModel& model,
                                                                const VotingConfig& config) const
{
    std::vector<const TemplatePoint*> valid;
    valid.reserve(model.points.size());
    for (const TemplatePoint& point : model.points) {
        if (norm(gradientDirection(point)) > 1e-6) {
            valid.push_back(&point);
        }
    }
    const int maxPoints = std::max(1, config.maxTemplateVotePoints);
    if (static_cast<int>(valid.size()) <= maxPoints) {
        return valid;
    }

    std::map<int, std::vector<const TemplatePoint*>> byChain;
    for (const TemplatePoint* point : valid) {
        byChain[point->chainId].push_back(point);
    }
    std::vector<const TemplatePoint*> selected;
    selected.reserve(static_cast<size_t>(maxPoints));
    const int chainCount = std::max(1, static_cast<int>(byChain.size()));
    int remaining = maxPoints;
    for (auto it = byChain.begin(); it != byChain.end() && remaining > 0; ++it) {
        const int chainsLeft = static_cast<int>(std::distance(it, byChain.end()));
        const int quota = std::max(1, remaining / std::max(1, chainsLeft));
        const std::vector<const TemplatePoint*>& chain = it->second;
        const int take = std::min<int>(quota, static_cast<int>(chain.size()));
        if (take >= static_cast<int>(chain.size())) {
            selected.insert(selected.end(), chain.begin(), chain.end());
        } else {
            for (int i = 0; i < take; ++i) {
                const int idx = take <= 1 ? 0 : static_cast<int>(std::round(
                    static_cast<double>(i) * static_cast<double>(chain.size() - 1) / static_cast<double>(take - 1)));
                selected.push_back(chain[static_cast<size_t>(std::clamp(idx, 0, static_cast<int>(chain.size()) - 1))]);
            }
        }
        remaining = maxPoints - static_cast<int>(selected.size());
    }
    if (static_cast<int>(selected.size()) > maxPoints) {
        selected.resize(static_cast<size_t>(maxPoints));
    }
    (void)chainCount;
    return selected;
}

} // namespace ShapeMatch
