#include "shape_match/coarse/SparseAccumulator.h"

#include <algorithm>
#include <cmath>

namespace ShapeMatch {

size_t AccKeyHash::operator()(const AccKey& key) const
{
    size_t h = static_cast<size_t>(key.xBin) * 73856093u;
    h ^= static_cast<size_t>(key.yBin) * 19349663u;
    h ^= static_cast<size_t>(key.thetaBin) * 83492791u;
    return h;
}

SparseAccumulator::SparseAccumulator(VotingConfig config)
    : m_config(config)
{
}

void SparseAccumulator::clear()
{
    m_bins.clear();
    m_totalVoteCount = 0;
}

void SparseAccumulator::vote(float centerX, float centerY, int thetaBin, float weight)
{
    if (!std::isfinite(centerX) || !std::isfinite(centerY) || centerX < 0.0f || centerY < 0.0f) {
        return;
    }
    const float cell = std::max(0.5f, m_config.accumulatorCellSizePx);
    const int xBin = static_cast<int>(std::floor(centerX / cell));
    const int yBin = static_cast<int>(std::floor(centerY / cell));
    const int binCount = std::max(1, m_config.orientationBinCount);
    thetaBin %= binCount;
    if (thetaBin < 0) {
        thetaBin += binCount;
    }

    AccValue& value = m_bins[AccKey{xBin, yBin, thetaBin}];
    value.score += std::max(0.0f, weight);
    value.voteCount += 1;
    ++m_totalVoteCount;
}

std::vector<VotePeak> SparseAccumulator::findPeaks(int topK, cv::Size imageSize) const
{
    std::vector<VotePeak> candidates;
    candidates.reserve(m_bins.size());
    const float cell = std::max(0.5f, m_config.accumulatorCellSizePx);
    const int binCount = std::max(1, m_config.orientationBinCount);
    for (const auto& item : m_bins) {
        const AccKey& key = item.first;
        const AccValue& value = item.second;
        if (value.voteCount < m_config.minVoteCount || value.score < m_config.minVoteScore) {
            continue;
        }
        const double x = (static_cast<double>(key.xBin) + 0.5) * cell;
        const double y = (static_cast<double>(key.yBin) + 0.5) * cell;
        if (x < 0.0 || y < 0.0 || x >= imageSize.width || y >= imageSize.height) {
            continue;
        }
        VotePeak peak;
        peak.pose.x = x;
        peak.pose.y = y;
        peak.pose.theta = 2.0 * kPi * static_cast<double>(key.thetaBin) / static_cast<double>(binCount);
        peak.pose.theta = wrapToPi(peak.pose.theta);
        peak.pose.scale = 1.0;
        peak.voteScore = value.score;
        peak.voteCount = value.voteCount;
        peak.xBin = key.xBin;
        peak.yBin = key.yBin;
        peak.thetaBin = key.thetaBin;
        candidates.push_back(peak);
    }

    std::sort(candidates.begin(), candidates.end(), [](const VotePeak& a, const VotePeak& b) {
        if (a.voteScore != b.voteScore) {
            return a.voteScore > b.voteScore;
        }
        return a.voteCount > b.voteCount;
    });

    std::vector<VotePeak> peaks;
    peaks.reserve(static_cast<size_t>(std::max(0, topK)));
    for (const VotePeak& candidate : candidates) {
        if (isSuppressedByKept(candidate, peaks)) {
            continue;
        }
        peaks.push_back(candidate);
        if (static_cast<int>(peaks.size()) >= topK) {
            break;
        }
    }
    return peaks;
}

int SparseAccumulator::nonZeroBinCount() const
{
    return static_cast<int>(m_bins.size());
}

int SparseAccumulator::totalVoteCount() const
{
    return m_totalVoteCount;
}

bool SparseAccumulator::isSuppressedByKept(const VotePeak& candidate, const std::vector<VotePeak>& kept) const
{
    for (const VotePeak& peak : kept) {
        if (std::abs(candidate.xBin - peak.xBin) <= m_config.accumulatorNmsRadiusXY
            && std::abs(candidate.yBin - peak.yBin) <= m_config.accumulatorNmsRadiusXY
            && thetaBinDistance(candidate.thetaBin, peak.thetaBin) <= m_config.accumulatorNmsRadiusTheta) {
            return true;
        }
    }
    return false;
}

int SparseAccumulator::thetaBinDistance(int a, int b) const
{
    const int binCount = std::max(1, m_config.orientationBinCount);
    int d = std::abs(a - b) % binCount;
    return std::min(d, binCount - d);
}

} // namespace ShapeMatch
