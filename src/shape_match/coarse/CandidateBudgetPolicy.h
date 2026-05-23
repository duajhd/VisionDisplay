#pragma once

#include "shape_match/coarse/CoarseCandidate.h"
#include "shape_match/coarse/CoarseMatchConfig.h"

#include <opencv2/core.hpp>

#include <vector>

namespace ShapeMatch {

struct RefineWindow
{
    int radiusPx = 0;
    double angleRadiusDeg = 0.0;
    int maxChildren = 0;
};

struct ParentSelectionResult
{
    std::vector<CoarseCandidate> parents;
    int beforeCount = 0;
    int afterCount = 0;
};

class CandidateBudgetPolicy
{
public:
    explicit CandidateBudgetPolicy(CoarseMatchConfig config = {});

    ParentSelectionResult selectParents(const std::vector<CoarseCandidate>& candidates,
                                        const cv::Size& imageSize,
                                        int level) const;

    RefineWindow refineWindowForParent(const CoarseCandidate& parent, int level) const;
    int candidateBudgetForLevel(int level) const;
    int maxParentsForLevel(int level) const;

private:
    bool sameParent(const CoarseCandidate& a, const CoarseCandidate& b, int level) const;
    int cellIndex(const MatchPose& pose, const cv::Size& imageSize) const;

    CoarseMatchConfig m_config;
};

} // namespace ShapeMatch
