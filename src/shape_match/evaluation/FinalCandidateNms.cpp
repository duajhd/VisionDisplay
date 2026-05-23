#include "shape_match/evaluation/FinalCandidateNms.h"

#include <algorithm>
#include <cmath>

namespace ShapeMatch {

FinalCandidateNms::FinalCandidateNms(ShapeMatchEvalConfig config)
    : m_config(config)
{
}

FinalCandidateNmsResult FinalCandidateNms::apply(const std::vector<MatchPose>& candidates) const
{
    FinalCandidateNmsResult result;
    result.inputCount = static_cast<int>(candidates.size());

    if (!m_config.enableFinalRankerCandidateNms) {
        result.poses = candidates;
    } else {
        const int hardLimit = m_config.maxFinalRankerInputCandidates > 0
            ? m_config.maxFinalRankerInputCandidates
            : static_cast<int>(candidates.size());
        const int targetLimit = m_config.targetFinalRankerInputCandidates > 0
            ? std::min(m_config.targetFinalRankerInputCandidates, hardLimit)
            : hardLimit;
        result.poses.reserve(static_cast<size_t>(std::min<int>(hardLimit, candidates.size())));
        for (const MatchPose& pose : candidates) {
            bool duplicate = false;
            for (const MatchPose& kept : result.poses) {
                if (isDuplicate(pose, kept)) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) {
                result.poses.push_back(pose);
                if (static_cast<int>(result.poses.size()) >= hardLimit) {
                    break;
                }
            }
        }
        if (targetLimit > 0 && static_cast<int>(result.poses.size()) > targetLimit) {
            result.poses.resize(static_cast<size_t>(targetLimit));
        }
    }

    result.afterNmsCount = static_cast<int>(result.poses.size());
    return result;
}

bool FinalCandidateNms::isDuplicate(const MatchPose& a, const MatchPose& b) const
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dxy = std::sqrt(dx * dx + dy * dy);
    const double dtheta = std::abs(radToDeg(wrapToPi(a.theta - b.theta)));
    const double dscale = std::abs(a.scale - b.scale);
    return dxy < m_config.finalNmsTranslationPx
        && dtheta < m_config.finalNmsAngleDeg
        && dscale < m_config.finalNmsScale;
}

} // namespace ShapeMatch
