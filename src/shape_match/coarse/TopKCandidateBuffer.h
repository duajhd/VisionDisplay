#pragma once

#include "shape_match/coarse/CoarseCandidate.h"

#include <vector>

namespace ShapeMatch {

class TopKCandidateBuffer
{
public:
    TopKCandidateBuffer(int maxSize,
                        double nmsTranslationThresholdPx,
                        double nmsAngleThresholdDeg);

    void add(const CoarseCandidate& candidate);
    std::vector<CoarseCandidate> sortedCandidates() const;
    bool hasEnough() const;
    double minScore() const;
    int size() const;
    const std::vector<CoarseCandidate>& suppressedCandidates() const;

private:
    bool isSamePose(const CoarseCandidate& a, const CoarseCandidate& b) const;
    void sortAndTrim();

    int m_maxSize = 0;
    double m_nmsTranslationThresholdPx = 4.0;
    double m_nmsAngleThresholdDeg = 3.0;
    std::vector<CoarseCandidate> m_candidates;
    std::vector<CoarseCandidate> m_suppressedCandidates;
};

} // namespace ShapeMatch
