#include "shape_match/coarse/SpatialDiversityTopKBuffer.h"

#include <algorithm>
#include <cmath>

namespace ShapeMatch {

SpatialDiversityTopKBuffer::SpatialDiversityTopKBuffer(const CoarseMatchConfig& config,
                                                       int imageWidth,
                                                       int imageHeight,
                                                       int outputTopK,
                                                       double nmsTranslationThresholdPx,
                                                       double nmsAngleThresholdDeg)
    : m_config(config)
    , m_imageWidth(std::max(1, imageWidth))
    , m_imageHeight(std::max(1, imageHeight))
    , m_outputTopK(std::max(1, outputTopK))
    , m_nmsTranslationThresholdPx(std::max(0.0, nmsTranslationThresholdPx))
    , m_nmsAngleThresholdDeg(std::max(0.0, nmsAngleThresholdDeg))
{
    m_config.diversityGridRows = std::max(1, m_config.diversityGridRows);
    m_config.diversityGridCols = std::max(1, m_config.diversityGridCols);
    m_config.topKPerGridCell = std::max(1, m_config.topKPerGridCell);
    m_cells.resize(static_cast<size_t>(m_config.diversityGridRows * m_config.diversityGridCols));
}

void SpatialDiversityTopKBuffer::add(const CoarseCandidate& candidate)
{
    if (candidateBeamScore(candidate) <= 0.0) {
        return;
    }
    ++m_rawCandidateCount;
    const int index = cellIndex(candidate.pose.x, candidate.pose.y);
    CoarseCandidate item = candidate;
    item.sourceCellRow = index / m_config.diversityGridCols;
    item.sourceCellCol = index % m_config.diversityGridCols;

    std::vector<CoarseCandidate>& cell = m_cells[static_cast<size_t>(index)];
    for (CoarseCandidate& existing : cell) {
            if (samePose(existing, item)) {
                if (candidateBeamScore(item) > candidateBeamScore(existing)) {
                    CoarseCandidate suppressed = existing;
                    suppressed.suppressedByNms = true;
                    suppressed.suppressorCandidateId = item.traceCandidateId;
                    suppressed.rejectReason = "nms_suppressed";
                    m_suppressedCandidates.push_back(std::move(suppressed));
                    existing = item;
                } else {
                    CoarseCandidate suppressed = item;
                    suppressed.suppressedByNms = true;
                    suppressed.suppressorCandidateId = existing.traceCandidateId;
                    suppressed.rejectReason = "nms_suppressed";
                    m_suppressedCandidates.push_back(std::move(suppressed));
                }
            sortDescending(cell);
            if (static_cast<int>(cell.size()) > m_config.topKPerGridCell) {
                cell.resize(static_cast<size_t>(m_config.topKPerGridCell));
            }
            return;
        }
    }

    cell.push_back(item);
    sortDescending(cell);
    if (static_cast<int>(cell.size()) > m_config.topKPerGridCell) {
        cell.resize(static_cast<size_t>(m_config.topKPerGridCell));
    }
}

std::vector<CoarseCandidate> SpatialDiversityTopKBuffer::sortedCandidates()
{
    m_suppressedCandidates.clear();
    std::vector<CoarseCandidate> merged;
    for (const std::vector<CoarseCandidate>& cell : m_cells) {
        merged.insert(merged.end(), cell.begin(), cell.end());
    }
    m_nmsBeforeCount = static_cast<int>(merged.size());
    const int globalLimit = std::max(m_outputTopK, m_config.globalTopKAfterDiversity > 0
                                                   ? std::min(m_config.globalTopKAfterDiversity, m_outputTopK)
                                                   : m_outputTopK);
    std::vector<CoarseCandidate> out = nmsAndTrim(std::move(merged), globalLimit);
    if (static_cast<int>(out.size()) > m_outputTopK) {
        out.resize(static_cast<size_t>(m_outputTopK));
    }
    m_nmsAfterCount = static_cast<int>(out.size());
    return out;
}

bool SpatialDiversityTopKBuffer::hasEnough() const
{
    int count = 0;
    for (const auto& cell : m_cells) {
        count += static_cast<int>(cell.size());
    }
    return count >= m_outputTopK;
}

double SpatialDiversityTopKBuffer::minScore() const
{
    double minValue = 0.0;
    bool have = false;
    for (const auto& cell : m_cells) {
        for (const CoarseCandidate& candidate : cell) {
            if (!have || candidateBeamScore(candidate) < minValue) {
                minValue = candidateBeamScore(candidate);
                have = true;
            }
        }
    }
    return have ? minValue : 0.0;
}

int SpatialDiversityTopKBuffer::rawCandidateCount() const
{
    return m_rawCandidateCount;
}

int SpatialDiversityTopKBuffer::nmsBeforeCount() const
{
    return m_nmsBeforeCount;
}

int SpatialDiversityTopKBuffer::nmsAfterCount() const
{
    return m_nmsAfterCount;
}

const std::vector<CoarseCandidate>& SpatialDiversityTopKBuffer::suppressedCandidates() const
{
    return m_suppressedCandidates;
}

std::vector<CoarseMatchCellStat> SpatialDiversityTopKBuffer::cellStats() const
{
    std::vector<CoarseMatchCellStat> stats;
    stats.reserve(m_cells.size());
    for (int row = 0; row < m_config.diversityGridRows; ++row) {
        for (int col = 0; col < m_config.diversityGridCols; ++col) {
            const int index = row * m_config.diversityGridCols + col;
            CoarseMatchCellStat stat;
            stat.row = row;
            stat.col = col;
            stat.keptCount = static_cast<int>(m_cells[static_cast<size_t>(index)].size());
            stat.candidateCount = stat.keptCount;
            stats.push_back(stat);
        }
    }
    return stats;
}

int SpatialDiversityTopKBuffer::cellIndex(double x, double y) const
{
    const int col = std::clamp(static_cast<int>(x / static_cast<double>(m_imageWidth) * m_config.diversityGridCols),
                               0,
                               m_config.diversityGridCols - 1);
    const int row = std::clamp(static_cast<int>(y / static_cast<double>(m_imageHeight) * m_config.diversityGridRows),
                               0,
                               m_config.diversityGridRows - 1);
    return row * m_config.diversityGridCols + col;
}

bool SpatialDiversityTopKBuffer::samePose(const CoarseCandidate& a, const CoarseCandidate& b) const
{
    return std::abs(a.pose.x - b.pose.x) < m_nmsTranslationThresholdPx
        && std::abs(a.pose.y - b.pose.y) < m_nmsTranslationThresholdPx
        && std::abs(radToDeg(wrapToPi(a.pose.theta - b.pose.theta))) < m_nmsAngleThresholdDeg
        && std::abs(a.pose.scale - b.pose.scale) < m_config.nmsScaleThreshold;
}

void SpatialDiversityTopKBuffer::sortDescending(std::vector<CoarseCandidate>& candidates)
{
    std::sort(candidates.begin(), candidates.end(), [](const CoarseCandidate& a, const CoarseCandidate& b) {
        return candidateBeamScore(a) > candidateBeamScore(b);
    });
}

std::vector<CoarseCandidate> SpatialDiversityTopKBuffer::nmsAndTrim(std::vector<CoarseCandidate> candidates, int limit) const
{
    sortDescending(candidates);
    std::vector<CoarseCandidate> out;
    out.reserve(static_cast<size_t>(std::max(1, limit)));
    for (const CoarseCandidate& candidate : candidates) {
        bool duplicate = false;
        for (const CoarseCandidate& kept : out) {
            if (samePose(candidate, kept)) {
                CoarseCandidate suppressed = candidate;
                suppressed.suppressedByNms = true;
                suppressed.suppressorCandidateId = kept.traceCandidateId;
                suppressed.rejectReason = "nms_suppressed";
                m_suppressedCandidates.push_back(std::move(suppressed));
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            out.push_back(candidate);
            if (static_cast<int>(out.size()) >= limit) {
                break;
            }
        }
    }
    return out;
}

} // namespace ShapeMatch
