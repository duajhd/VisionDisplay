#include "shape_match/pipeline_v2/ResponsePeakCandidateSelector.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace ShapeMatch {

namespace {

double elapsed(std::chrono::steady_clock::time_point start)
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

double thetaDiffDeg(double a, double b)
{
    return std::abs(radToDeg(wrapToPi(a - b)));
}

double symmetryAwareThetaDiffDeg(double a, double b, const ShapeMatchPipelineV2Config& config)
{
    double diff = thetaDiffDeg(a, b);
    if (config.symmetry.enableSymmetrySuppression &&
        config.symmetry.assume180DegreeSymmetry &&
        config.symmetry.symmetryAngleDeg > 1e-6) {
        const double period = std::clamp(config.symmetry.symmetryAngleDeg, 1e-6, 360.0);
        const double reduced = std::fmod(diff, period);
        diff = std::min(reduced, period - reduced);
    }
    return diff;
}

bool suppresses(const VerifiedResponsePeak& a, const VerifiedResponsePeak& b, const ShapeMatchPipelineV2Config& config)
{
    const double dx = a.pose.x - b.pose.x;
    const double dy = a.pose.y - b.pose.y;
    const double dxy = std::sqrt(dx * dx + dy * dy);
    const double angleThreshold = config.symmetry.enableSymmetrySuppression && config.symmetry.assume180DegreeSymmetry
        ? std::max(config.responsePipeline.responseCandidateNmsThetaDeg, config.symmetry.symmetryNmsAngleToleranceDeg)
        : config.responsePipeline.responseCandidateNmsThetaDeg;
    const double xyThreshold = config.symmetry.enableSymmetrySuppression && config.symmetry.assume180DegreeSymmetry
        ? std::max(config.responsePipeline.responseCandidateNmsPx, config.symmetry.symmetryNmsTranslationPx)
        : config.responsePipeline.responseCandidateNmsPx;
    return dxy < xyThreshold &&
           symmetryAwareThetaDiffDeg(a.pose.theta, b.pose.theta, config) < angleThreshold;
}

int tileIndex(double value, int extent, int tileCount)
{
    if (extent <= 0 || tileCount <= 1) {
        return 0;
    }
    return std::clamp(static_cast<int>(value * tileCount / static_cast<double>(extent)), 0, tileCount - 1);
}

cv::Size imageSizeAt(const ShapeMatchPipelineV2Context& context, int level)
{
    if (context.imagePyramid != nullptr && level >= 0 && level < context.imagePyramid->levelCount()) {
        return context.imagePyramid->level(level).imageSize;
    }
    return context.edgeData != nullptr ? context.edgeData->imageSize : cv::Size();
}

} // namespace

std::vector<VerifiedResponsePeak> ResponsePeakCandidateSelector::select(const std::vector<VerifiedResponsePeak>& verifiedPeaks,
                                                                        const ShapeMatchPipelineV2Context& context,
                                                                        const ShapeMatchPipelineV2Config& config,
                                                                        double* elapsedMs) const
{
    const auto start = std::chrono::steady_clock::now();
    std::vector<VerifiedResponsePeak> sorted;
    sorted.reserve(verifiedPeaks.size());
    double maxResponse = 0.0;
    for (const VerifiedResponsePeak& peak : verifiedPeaks) {
        if (peak.accepted) {
            sorted.push_back(peak);
            maxResponse = std::max(maxResponse, peak.responseScore);
        }
    }
    if (maxResponse <= 1e-9) {
        maxResponse = 1.0;
    }
    for (VerifiedResponsePeak& peak : sorted) {
        const double normalizedResponse = peak.responseScore / maxResponse;
        peak.combinedScore = config.responsePipeline.responseWeight * normalizedResponse +
                             config.responsePipeline.fastScoreWeight * peak.fastScore;
    }
    std::sort(sorted.begin(), sorted.end(), [](const VerifiedResponsePeak& a, const VerifiedResponsePeak& b) {
        return a.combinedScore > b.combinedScore;
    });

    const int rows = std::max(1, config.responsePipeline.responseCandidateTileRows);
    const int cols = std::max(1, config.responsePipeline.responseCandidateTileCols);
    const cv::Size imageSize = imageSizeAt(context, config.responsePipeline.responsePyramidLevel);
    std::vector<int> tileCounts(static_cast<size_t>(rows * cols), 0);

    std::vector<VerifiedResponsePeak> selected;
    selected.reserve(static_cast<size_t>(config.responsePipeline.maxFinalCandidates));
    for (const VerifiedResponsePeak& peak : sorted) {
        if (static_cast<int>(selected.size()) >= config.responsePipeline.maxFinalCandidates) {
            break;
        }

        if (config.responsePipeline.enableResponsePeakNms) {
            bool suppressed = false;
            for (const VerifiedResponsePeak& existing : selected) {
                if (suppresses(existing, peak, config)) {
                    suppressed = true;
                    break;
                }
            }
            if (suppressed) {
                continue;
            }
        }

        const int row = tileIndex(peak.pose.y, imageSize.height, rows);
        const int col = tileIndex(peak.pose.x, imageSize.width, cols);
        const int tile = row * cols + col;
        if (config.responsePipeline.enableResponsePeakTileDiversity &&
            tile >= 0 && tile < static_cast<int>(tileCounts.size()) &&
            tileCounts[static_cast<size_t>(tile)] >= config.responsePipeline.maxCandidatesPerTile) {
            continue;
        }

        selected.push_back(peak);
        if (tile >= 0 && tile < static_cast<int>(tileCounts.size())) {
            ++tileCounts[static_cast<size_t>(tile)];
        }
        if (static_cast<int>(selected.size()) >= config.responsePipeline.targetFinalCandidates &&
            static_cast<int>(selected.size()) >= config.responsePipeline.maxFinalCandidates) {
            break;
        }
    }

    if (elapsedMs != nullptr) {
        *elapsedMs = elapsed(start);
    }
    return selected;
}

} // namespace ShapeMatch
