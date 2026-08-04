#include "shape_match/coarse/ResponsePeakExtractor.h"

#include <algorithm>
#include <cmath>

namespace ShapeMatch {

namespace {

int tileIndex(int value, int extent, int tileCount)
{
    if (tileCount <= 1 || extent <= 0) {
        return 0;
    }
    return std::clamp(static_cast<int>(static_cast<double>(value) * tileCount / extent), 0, tileCount - 1);
}

int thetaBinDistance(int a, int b, int count)
{
    const int d = std::abs(a - b);
    return std::min(d, std::max(0, count - d));
}

bool suppresses(const ResponsePeak& kept, const ResponsePeak& candidate, const OrientationResponseConfig& config)
{
    const double dx = kept.pose.x - candidate.pose.x;
    const double dy = kept.pose.y - candidate.pose.y;
    const double dxy = std::sqrt(dx * dx + dy * dy);
    return dxy < static_cast<double>(config.nmsRadiusPx) &&
           thetaBinDistance(kept.thetaBin, candidate.thetaBin, std::max(1, config.thetaBinCount)) <=
               config.nmsThetaRadiusBins;
}

} // namespace

std::vector<ResponsePeak> ResponsePeakExtractor::extractPeaks(const cv::Mat& responseXY,
                                                              int thetaBin,
                                                              double thetaRad,
                                                              const OrientationResponseConfig& config) const
{
    std::vector<ResponsePeak> peaks;
    if (responseXY.empty() || responseXY.type() != CV_32F) {
        return peaks;
    }

    const int rows = std::max(1, config.tileRows);
    const int cols = std::max(1, config.tileCols);
    std::vector<int> tileCounts(static_cast<size_t>(rows * cols), 0);

    for (int y = 1; y < responseXY.rows - 1; ++y) {
        const float* prev = responseXY.ptr<float>(y - 1);
        const float* curr = responseXY.ptr<float>(y);
        const float* next = responseXY.ptr<float>(y + 1);
        for (int x = 1; x < responseXY.cols - 1; ++x) {
            const float score = curr[x];
            if (score < config.minResponseScore) {
                continue;
            }
            if (score < curr[x - 1] || score < curr[x + 1] ||
                score < prev[x - 1] || score < prev[x] || score < prev[x + 1] ||
                score < next[x - 1] || score < next[x] || score < next[x + 1]) {
                continue;
            }

            ResponsePeak peak;
            peak.pose = MatchPose{static_cast<double>(x), static_cast<double>(y), thetaRad, 1.0};
            peak.responseScore = score;
            peak.thetaBin = thetaBin;
            peak.x = x;
            peak.y = y;
            peak.tileRow = tileIndex(y, responseXY.rows, rows);
            peak.tileCol = tileIndex(x, responseXY.cols, cols);
            peaks.push_back(peak);
        }
    }

    std::sort(peaks.begin(), peaks.end(), [](const ResponsePeak& a, const ResponsePeak& b) {
        return a.responseScore > b.responseScore;
    });

    std::vector<ResponsePeak> kept;
    kept.reserve(std::min<int>(config.maxPeaksPerTheta, static_cast<int>(peaks.size())));
    std::fill(tileCounts.begin(), tileCounts.end(), 0);
    for (const ResponsePeak& peak : peaks) {
        if (static_cast<int>(kept.size()) >= config.maxPeaksPerTheta) {
            break;
        }
        const int tile = peak.tileRow * cols + peak.tileCol;
        if (config.useTilePeakQuota && tile >= 0 && tile < static_cast<int>(tileCounts.size()) &&
            tileCounts[static_cast<size_t>(tile)] >= config.maxPeaksPerTile) {
            continue;
        }
        bool suppressed = false;
        for (const ResponsePeak& existing : kept) {
            if (suppresses(existing, peak, config)) {
                suppressed = true;
                break;
            }
        }
        if (suppressed) {
            continue;
        }
        kept.push_back(peak);
        if (tile >= 0 && tile < static_cast<int>(tileCounts.size())) {
            ++tileCounts[static_cast<size_t>(tile)];
        }
    }
    return kept;
}

std::vector<ResponsePeak> ResponsePeakExtractor::mergeAndNms(const std::vector<ResponsePeak>& peaks,
                                                             const OrientationResponseConfig& config) const
{
    std::vector<ResponsePeak> sorted = peaks;
    std::sort(sorted.begin(), sorted.end(), [](const ResponsePeak& a, const ResponsePeak& b) {
        return a.responseScore > b.responseScore;
    });

    std::vector<ResponsePeak> kept;
    kept.reserve(std::min<int>(config.maxTotalPeaks, static_cast<int>(sorted.size())));
    for (const ResponsePeak& peak : sorted) {
        if (static_cast<int>(kept.size()) >= config.maxTotalPeaks) {
            break;
        }
        bool suppressed = false;
        for (const ResponsePeak& existing : kept) {
            if (suppresses(existing, peak, config)) {
                suppressed = true;
                break;
            }
        }
        if (!suppressed) {
            kept.push_back(peak);
        }
    }
    return kept;
}

} // namespace ShapeMatch
