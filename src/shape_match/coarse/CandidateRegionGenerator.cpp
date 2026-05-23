#include "shape_match/coarse/CandidateRegionGenerator.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace ShapeMatch {

CandidateRegionGenerator::CandidateRegionGenerator(CoarseMatchConfig config)
    : m_config(std::move(config))
{
}

std::vector<CandidateRegion> CandidateRegionGenerator::generateLevelRegions(
    const std::vector<CoarseCandidate>& seedCandidates,
    const ShapeTemplateModel& templateLevel,
    cv::Size imageSize,
    int level) const
{
    std::vector<CandidateRegion> regions;
    if (seedCandidates.empty() || imageSize.empty()) {
        return regions;
    }

    const cv::Rect2d bbox = templateLevel.boundingBox.area() > 0.0
        ? templateLevel.boundingBox
        : cv::Rect2d(-64.0, -64.0, 128.0, 128.0);
    const int baseW = std::max(64, static_cast<int>(std::ceil(bbox.width)));
    const int baseH = std::max(64, static_cast<int>(std::ceil(bbox.height)));
    const int padding = level <= 0 ? m_config.roiDistanceField.roiPaddingPxLevel0
                                   : m_config.roiDistanceField.roiPaddingPxLevel1;
    const int maxRegions = std::max(1, m_config.roiDistanceField.maxRoiDistanceFields);
    regions.reserve(static_cast<size_t>(std::min<int>(maxRegions, seedCandidates.size())));

    for (const CoarseCandidate& c : seedCandidates) {
        if (static_cast<int>(regions.size()) >= maxRegions) {
            break;
        }
        const int w = std::min(m_config.roiDistanceField.maxRoiWidth, std::max(m_config.roiDistanceField.minRoiWidth, baseW + 2 * padding + 2 * m_config.localRefineRadiusPx));
        const int h = std::min(m_config.roiDistanceField.maxRoiHeight, std::max(m_config.roiDistanceField.minRoiHeight, baseH + 2 * padding + 2 * m_config.localRefineRadiusPx));
        cv::Rect roi(static_cast<int>(std::round(c.pose.x - w * 0.5)),
                     static_cast<int>(std::round(c.pose.y - h * 0.5)),
                     w,
                     h);
        roi &= cv::Rect(0, 0, imageSize.width, imageSize.height);
        if (roi.empty()) {
            continue;
        }
        CandidateRegion r;
        r.roi = roi;
        r.score = candidateBeamScore(c);
        r.sourceCandidateCount = 1;
        regions.push_back(r);
    }
    return regions;
}

} // namespace ShapeMatch
