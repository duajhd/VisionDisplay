#include "shape_match/coarse/CandidateBudgetPolicy.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace ShapeMatch {

CandidateBudgetPolicy::CandidateBudgetPolicy(CoarseMatchConfig config)
    : m_config(std::move(config))
{
}

ParentSelectionResult CandidateBudgetPolicy::selectParents(const std::vector<CoarseCandidate>& candidates,
                                                           const cv::Size& imageSize,
                                                           int level) const
{
    ParentSelectionResult result;
    result.beforeCount = static_cast<int>(candidates.size());
    if (candidates.empty()) {
        return result;
    }
    std::vector<CoarseCandidate> sorted = candidates;
    std::sort(sorted.begin(), sorted.end(), [](const CoarseCandidate& a, const CoarseCandidate& b) {
        return a.fastScore > b.fastScore;
    });

    const int rows = std::max(1, m_config.parentDiversityGridRows);
    const int cols = std::max(1, m_config.parentDiversityGridCols);
    const int perCell = std::max(1, m_config.maxParentsPerGridCell);
    const int maxParents = maxParentsForLevel(level);
    std::vector<int> cellCounts(static_cast<size_t>(rows * cols), 0);
    result.parents.reserve(static_cast<size_t>(maxParents));

    for (const CoarseCandidate& candidate : sorted) {
        if (static_cast<int>(result.parents.size()) >= maxParents) {
            break;
        }
        bool duplicate = false;
        for (const CoarseCandidate& kept : result.parents) {
            if (sameParent(candidate, kept, level)) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) {
            continue;
        }
        const int cell = cellIndex(candidate.pose, imageSize);
        if (m_config.enableCandidateBudgetPolicy
            && m_config.enableParentDiversityBeforeRefine
            && cellCounts[static_cast<size_t>(cell)] >= perCell) {
            continue;
        }
        result.parents.push_back(candidate);
        ++cellCounts[static_cast<size_t>(cell)];
    }

    if (result.parents.empty()) {
        result.parents.push_back(sorted.front());
    }
    result.afterCount = static_cast<int>(result.parents.size());
    return result;
}

RefineWindow CandidateBudgetPolicy::refineWindowForParent(const CoarseCandidate& parent, int level) const
{
    RefineWindow out;
    const bool level0 = level <= 0;
    const bool high = parent.fastScore >= m_config.highConfidenceScore;
    const bool medium = parent.fastScore >= m_config.mediumConfidenceScore;

    if (!m_config.enableAdaptiveRefineWindow) {
        out.radiusPx = std::max(0, m_config.localRefineRadiusPx);
        out.angleRadiusDeg = std::max(0.0, static_cast<double>(m_config.localRefineAngleRadiusDeg));
    } else if (level0) {
        out.radiusPx = high ? m_config.level0HighConfRadiusPx
                            : (medium ? m_config.level0MediumConfRadiusPx : m_config.level0LowConfRadiusPx);
        out.angleRadiusDeg = high ? m_config.level0HighConfAngleRadiusDeg
                                  : (medium ? m_config.level0MediumConfAngleRadiusDeg : m_config.level0LowConfAngleRadiusDeg);
    } else {
        out.radiusPx = high ? m_config.coarseHighConfRadiusPx
                            : (medium ? m_config.coarseMediumConfRadiusPx : m_config.coarseLowConfRadiusPx);
        out.angleRadiusDeg = high ? m_config.level0LowConfAngleRadiusDeg
                                  : static_cast<double>(m_config.localRefineAngleRadiusDeg);
    }

    out.radiusPx = std::clamp(out.radiusPx, 0, std::max(0, m_config.localRefineRadiusPx));
    out.angleRadiusDeg = std::min(out.angleRadiusDeg, static_cast<double>(std::max(0, m_config.localRefineAngleRadiusDeg)));
    out.maxChildren = level0 ? std::max(1, m_config.maxChildrenPerParentLevel0)
                             : std::max(1, m_config.maxChildrenPerParentCoarse);
    return out;
}

int CandidateBudgetPolicy::candidateBudgetForLevel(int level) const
{
    if (!m_config.enableCandidateBudgetPolicy || !m_config.enableAdaptiveCandidateBudget) {
        return std::max(1, m_config.maxCandidatesEvaluatedPerLevel);
    }
    if (level <= 0) {
        return std::max(1, std::min({m_config.maxCandidatesEvaluatedPerLevel,
                                     m_config.maxCandidatesLocalRefine,
                                     m_config.maxCandidatesLevel0}));
    }
    return std::max(1, std::min(m_config.maxCandidatesEvaluatedPerLevel, m_config.maxCandidatesPerLevel));
}

int CandidateBudgetPolicy::maxParentsForLevel(int level) const
{
    if (!m_config.enableCandidateBudgetPolicy || !m_config.enableAdaptiveCandidateBudget) {
        return std::max(1, m_config.beamWidth);
    }
    if (level <= 0) {
        return std::max(1, std::min(m_config.maxParentsForLevel0Refine, m_config.maxBeamForLocalRefineLevel0));
    }
    return std::max(1, std::min(m_config.beamWidth, m_config.maxBeamForLocalRefineCoarse));
}

bool CandidateBudgetPolicy::sameParent(const CoarseCandidate& a, const CoarseCandidate& b, int level) const
{
    const double xyThreshold = level <= 0 ? 3.0 : std::max(4.0, m_config.nmsTranslationThresholdPx);
    const double angleThreshold = std::max(2.0, m_config.nmsAngleThresholdDeg);
    return std::abs(a.pose.x - b.pose.x) < xyThreshold
        && std::abs(a.pose.y - b.pose.y) < xyThreshold
        && std::abs(radToDeg(wrapToPi(a.pose.theta - b.pose.theta))) < angleThreshold
        && std::abs(a.pose.scale - b.pose.scale) < m_config.nmsScaleThreshold;
}

int CandidateBudgetPolicy::cellIndex(const MatchPose& pose, const cv::Size& imageSize) const
{
    const int rows = std::max(1, m_config.parentDiversityGridRows);
    const int cols = std::max(1, m_config.parentDiversityGridCols);
    const int col = std::clamp(static_cast<int>(pose.x / std::max(1.0, static_cast<double>(imageSize.width)) * cols), 0, cols - 1);
    const int row = std::clamp(static_cast<int>(pose.y / std::max(1.0, static_cast<double>(imageSize.height)) * rows), 0, rows - 1);
    return row * cols + col;
}

} // namespace ShapeMatch
