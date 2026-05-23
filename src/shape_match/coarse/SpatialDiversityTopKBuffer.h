#pragma once

#include "shape_match/coarse/CoarseCandidate.h"
#include "shape_match/coarse/CoarseMatchConfig.h"

#include <vector>

namespace ShapeMatch {

class SpatialDiversityTopKBuffer
{
public:
    SpatialDiversityTopKBuffer(const CoarseMatchConfig& config,
                               int imageWidth,
                               int imageHeight,
                               int outputTopK,
                               double nmsTranslationThresholdPx,
                               double nmsAngleThresholdDeg);

    void add(const CoarseCandidate& candidate);
    std::vector<CoarseCandidate> sortedCandidates();
    bool hasEnough() const;
    double minScore() const;
    int rawCandidateCount() const;
    int nmsBeforeCount() const;
    int nmsAfterCount() const;
    const std::vector<CoarseCandidate>& suppressedCandidates() const;
    std::vector<CoarseMatchCellStat> cellStats() const;

private:
    int cellIndex(double x, double y) const;
    bool samePose(const CoarseCandidate& a, const CoarseCandidate& b) const;
    static void sortDescending(std::vector<CoarseCandidate>& candidates);
    std::vector<CoarseCandidate> nmsAndTrim(std::vector<CoarseCandidate> candidates, int limit) const;

    CoarseMatchConfig m_config;
    int m_imageWidth = 0;
    int m_imageHeight = 0;
    int m_outputTopK = 0;
    double m_nmsTranslationThresholdPx = 0.0;
    double m_nmsAngleThresholdDeg = 0.0;
    std::vector<std::vector<CoarseCandidate>> m_cells;
    int m_rawCandidateCount = 0;
    int m_nmsBeforeCount = 0;
    int m_nmsAfterCount = 0;
    mutable std::vector<CoarseCandidate> m_suppressedCandidates;
};

} // namespace ShapeMatch
