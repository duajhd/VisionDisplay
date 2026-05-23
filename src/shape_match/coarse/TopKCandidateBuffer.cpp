#include "shape_match/coarse/TopKCandidateBuffer.h"

#include <algorithm>
#include <cmath>

namespace ShapeMatch {

TopKCandidateBuffer::TopKCandidateBuffer(int maxSize,
                                         double nmsTranslationThresholdPx,
                                         double nmsAngleThresholdDeg)
    : m_maxSize(std::max(1, maxSize))
    , m_nmsTranslationThresholdPx(std::max(0.0, nmsTranslationThresholdPx))
    , m_nmsAngleThresholdDeg(std::max(0.0, nmsAngleThresholdDeg))
{
}

void TopKCandidateBuffer::add(const CoarseCandidate& candidate)
{
    if (candidateBeamScore(candidate) <= 0.0) {
        return;
    }
    for (CoarseCandidate& existing : m_candidates) {
        if (!isSamePose(existing, candidate)) {
            continue;
        }
        if (candidateBeamScore(candidate) > candidateBeamScore(existing)) {
            CoarseCandidate suppressed = existing;
            suppressed.suppressedByNms = true;
            suppressed.suppressorCandidateId = candidate.traceCandidateId;
            suppressed.rejectReason = "nms_suppressed";
            m_suppressedCandidates.push_back(std::move(suppressed));
            existing = candidate;
        } else {
            CoarseCandidate suppressed = candidate;
            suppressed.suppressedByNms = true;
            suppressed.suppressorCandidateId = existing.traceCandidateId;
            suppressed.rejectReason = "nms_suppressed";
            m_suppressedCandidates.push_back(std::move(suppressed));
        }
        sortAndTrim();
        return;
    }

    m_candidates.push_back(candidate);
    sortAndTrim();
}

std::vector<CoarseCandidate> TopKCandidateBuffer::sortedCandidates() const
{
    std::vector<CoarseCandidate> out = m_candidates;
    std::sort(out.begin(), out.end(), [](const CoarseCandidate& a, const CoarseCandidate& b) {
        return candidateBeamScore(a) > candidateBeamScore(b);
    });
    return out;
}

bool TopKCandidateBuffer::hasEnough() const
{
    return static_cast<int>(m_candidates.size()) >= m_maxSize;
}

double TopKCandidateBuffer::minScore() const
{
    if (m_candidates.empty()) {
        return 0.0;
    }
    const auto it = std::min_element(m_candidates.begin(), m_candidates.end(), [](const CoarseCandidate& a, const CoarseCandidate& b) {
        return candidateBeamScore(a) < candidateBeamScore(b);
    });
    return it == m_candidates.end() ? 0.0 : candidateBeamScore(*it);
}

int TopKCandidateBuffer::size() const
{
    return static_cast<int>(m_candidates.size());
}

const std::vector<CoarseCandidate>& TopKCandidateBuffer::suppressedCandidates() const
{
    return m_suppressedCandidates;
}

bool TopKCandidateBuffer::isSamePose(const CoarseCandidate& a, const CoarseCandidate& b) const
{
    return std::abs(a.pose.x - b.pose.x) < m_nmsTranslationThresholdPx
        && std::abs(a.pose.y - b.pose.y) < m_nmsTranslationThresholdPx
        && std::abs(radToDeg(wrapToPi(a.pose.theta - b.pose.theta))) < m_nmsAngleThresholdDeg
        && std::abs(a.pose.scale - b.pose.scale) < 1e-6;
}

void TopKCandidateBuffer::sortAndTrim()
{
    std::sort(m_candidates.begin(), m_candidates.end(), [](const CoarseCandidate& a, const CoarseCandidate& b) {
        return candidateBeamScore(a) > candidateBeamScore(b);
    });
    if (static_cast<int>(m_candidates.size()) > m_maxSize) {
        m_candidates.resize(static_cast<size_t>(m_maxSize));
    }
}

} // namespace ShapeMatch
