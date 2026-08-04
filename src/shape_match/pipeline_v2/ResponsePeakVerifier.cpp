#include "shape_match/pipeline_v2/ResponsePeakVerifier.h"

#include "shape_match/coarse/FastPoseScorer.h"

#include <algorithm>
#include <chrono>

namespace ShapeMatch {

namespace {

double elapsed(std::chrono::steady_clock::time_point start)
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

const ShapeTemplateModel* templateAt(const ShapeMatchPipelineV2Context& context, int level)
{
    if (context.templatePyramid != nullptr && level >= 0 && level < context.templatePyramid->levelCount()) {
        return &context.templatePyramid->level(level);
    }
    return level == 0 ? context.templateModel : nullptr;
}

const EdgeImageData* edgeAt(const ShapeMatchPipelineV2Context& context, int level)
{
    if (context.imagePyramid != nullptr && level >= 0 && level < context.imagePyramid->levelCount()) {
        return &context.imagePyramid->level(level);
    }
    return level == 0 ? context.edgeData : nullptr;
}

} // namespace

std::vector<VerifiedResponsePeak> ResponsePeakVerifier::verify(const std::vector<ResponsePeak>& peaks,
                                                               const ShapeMatchPipelineV2Context& context,
                                                               const ShapeMatchPipelineV2Config& config,
                                                               double* elapsedMs) const
{
    const auto start = std::chrono::steady_clock::now();
    std::vector<VerifiedResponsePeak> verified;
    if (elapsedMs != nullptr) {
        *elapsedMs = 0.0;
    }

    const int level = config.responsePipeline.responsePyramidLevel;
    const ShapeTemplateModel* model = templateAt(context, level);
    const EdgeImageData* edgeData = edgeAt(context, level);
    if (model == nullptr || edgeData == nullptr) {
        if (elapsedMs != nullptr) {
            *elapsedMs = elapsed(start);
        }
        return verified;
    }

    const int maxPeaks = std::min<int>(static_cast<int>(peaks.size()), std::max(1, config.responsePipeline.maxResponsePeaks));
    verified.reserve(static_cast<size_t>(maxPeaks));
    FastPoseScorer scorer;
    FastScoreContext scoreContext;

    for (int i = 0; i < maxPeaks; ++i) {
        const ResponsePeak& peak = peaks[static_cast<size_t>(i)];
        CoarseCandidate scored = scorer.scorePose(*model, *edgeData, peak.pose, level, scoreContext);
        VerifiedResponsePeak item;
        item.peak = peak;
        item.pose = peak.pose;
        item.responseScore = peak.responseScore;
        item.fastScore = scored.fastScore;
        item.coverageApprox = scored.coverageApprox;
        item.orientationApprox = scored.orientationApprox;
        item.polarityApprox = scored.polarityApprox;
        item.accepted = scored.fastScore >= config.responsePipeline.minVerifiedFastScore;
        item.rejectReason = item.accepted ? "" : "fast_score_below_threshold";
        verified.push_back(item);
    }

    std::sort(verified.begin(), verified.end(), [](const VerifiedResponsePeak& a, const VerifiedResponsePeak& b) {
        if (a.accepted != b.accepted) {
            return a.accepted > b.accepted;
        }
        return a.fastScore > b.fastScore;
    });
    if (static_cast<int>(verified.size()) > config.responsePipeline.maxVerifiedResponsePeaks) {
        verified.resize(static_cast<size_t>(config.responsePipeline.maxVerifiedResponsePeaks));
    }

    if (elapsedMs != nullptr) {
        *elapsedMs = elapsed(start);
    }
    return verified;
}

} // namespace ShapeMatch
