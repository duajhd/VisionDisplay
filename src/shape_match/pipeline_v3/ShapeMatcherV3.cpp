#include "shape_match/pipeline_v3/ShapeMatcherV3.h"

#include "shape_match/pipeline_v3/AngleViewBuilderV3.h"
#include "shape_match/pipeline_v3/CoarseSearchV3.h"
#include "shape_match/pipeline_v3/PoseRefinerV3.h"
#include "shape_match/pipeline_v3/ResponseMapBuilderV3.h"
#include "shape_match/pipeline_v3/WorkerPoolV3.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace ShapeMatch {
namespace {
using Clock = std::chrono::steady_clock;
double ms(Clock::time_point t) { return std::chrono::duration<double, std::milli>(Clock::now() - t).count(); }
}

ShapeModelV3 ShapeMatcherV3::createModel(const cv::Mat& image, const cv::Mat& mask,
                                         const ShapeModelParametersV3& p) const
{
    return ShapeModelTrainerV3().createModel(image, mask, p);
}

std::vector<MatchResultV3> ShapeMatcherV3::find(const cv::Mat& image, const ShapeModelV3& model,
                                                const ShapeSearchParametersV3& input,
                                                ShapeMatchStatisticsV3* outputStats,
                                                const ICandidateVerifierV3* verifier) const
{
    const auto totalT0 = Clock::now();
    if (image.empty() || model.empty()) throw std::invalid_argument("empty V3 image or model");
    ShapeSearchParametersV3 p = input;
    p.minScore = std::clamp(p.minScore, 0.0f, 1.0f);
    if (p.maxMatches <= 0 || p.coarseTopK <= 0 || p.tileWidth <= 0 || p.tileHeight <= 0)
        throw std::invalid_argument("invalid V3 search capacity or tile size");
    p.coarseTopK = std::max(p.coarseTopK, p.maxMatches);
    ShapeMatchStatisticsV3 stats;
    WorkerPoolV3& workerPool = WorkerPoolV3::shared();
    auto t0 = Clock::now();
    std::array<cv::Mat, kPyramidLevelCountV3> images;
    images[0] = image;
    for (int level = 1; level < kPyramidLevelCountV3; ++level)
        cv::pyrDown(images[static_cast<size_t>(level - 1)], images[static_cast<size_t>(level)]);
    stats.pyramidTimeMs = ms(t0);

    constexpr int coarseLevel = 3;
    constexpr float coarseScale = 0.125f;
    t0 = Clock::now();
    const ResponseMapV3 response = ResponseMapBuilderV3().build(
        images[coarseLevel], model.parameters.gradientLow, model.parameters.gradientHigh,
        model.parameters.nonMaximumSuppression,
        p.enableMultithreading ? &workerPool : nullptr);
    stats.responseMapTimeMs = ms(t0);
    stats.distanceFieldTimeMs = stats.responseMapTimeMs;

    t0 = Clock::now();
    PyramidLevelModelV3 levelModel;
    levelModel.scale = coarseScale;
    levelModel.stride = response.stride;
    levelModel.angleViews = AngleViewBuilderV3().bindPrecomputed(
        model, coarseLevel, levelModel.stride);
    stats.angleViewTimeMs = ms(t0);

    t0 = Clock::now();
    ShapeSearchParametersV3 coarseParameters = p;
    if (p.searchRoi.area() > 0) {
        coarseParameters.searchRoi = cv::Rect(
            static_cast<int>(std::floor(p.searchRoi.x * coarseScale)),
            static_cast<int>(std::floor(p.searchRoi.y * coarseScale)),
            static_cast<int>(std::ceil(p.searchRoi.width * coarseScale)),
            static_cast<int>(std::ceil(p.searchRoi.height * coarseScale)));
    }
    std::vector<MatchCandidateV3> candidates = CoarseSearchV3().search(
        response, levelModel, coarseParameters,
        p.enableStatistics ? &stats : nullptr, &workerPool);
    for (MatchCandidateV3& c : candidates) c.pyramidLevel = coarseLevel;
    stats.coarseSearchTimeMs = ms(t0);
    stats.pyramidTrackTimeMs = 0.0;

    CandidateCollectorV3::keepTopK(candidates, p.coarseTopK);
    PyramidLevelModelV3 finalModel = std::move(levelModel);
    const int coarseNmsCapacity = p.enableSubpixelRefinement
        ? std::max(p.maxRefineCandidates, p.maxMatches) : p.maxMatches;
    candidates = PoseNmsV3().apply(candidates, finalModel.angleViews,
        coarseNmsCapacity, std::max(1.0f, p.nmsDistance * coarseScale), p.nmsAngleRadians);
    stats.coarseCandidates = candidates.size();
    stats.candidatesAfterNms = candidates.size();
    std::vector<MatchResultV3> results;
    results.reserve(candidates.size());
    t0 = Clock::now();
    for (const MatchCandidateV3& coarseCandidate : candidates) {
        MatchCandidateV3 c = coarseCandidate;
        c.x = static_cast<int>(std::lround(c.x / coarseScale));
        c.y = static_cast<int>(std::lround(c.y / coarseScale));
        c.pyramidLevel = 0;
        MatchResultV3 r;
        r.pose.x = c.x; r.pose.y = c.y;
        r.pose.theta = finalModel.angleViews[static_cast<size_t>(c.angleIndex)].angleRadians;
        r.pose.scale = 1.0; r.score = c.score; r.rawCost = c.rawCost;
        if (!p.enableFinalVerification || !verifier || verifier->verify(c, r)) {
            results.push_back(r);
            ++stats.verifiedCandidates;
        }
    }
    stats.verifyTimeMs = ms(t0);
    if (p.enableSubpixelRefinement && !results.empty()) {
        t0 = Clock::now();
        const PoseRefinerV3 refiner;
        const FineEdgeMapV3 edgeMap = refiner.buildEdgeMap(image);
        const int refineTasks = p.enableMultithreading
            ? std::clamp(workerPool.threadCount(), 1, static_cast<int>(results.size())) : 1;
        if (refineTasks == 1) {
            for (MatchResultV3& result : results)
                result = refiner.refine(edgeMap, model, result, p);
        } else {
            workerPool.parallelFor(0, static_cast<int>(results.size()), refineTasks,
                [&](int begin, int end, int) {
                    for (int index = begin; index < end; ++index)
                        results[static_cast<size_t>(index)] = refiner.refine(
                            edgeMap, model, results[static_cast<size_t>(index)], p);
                });
        }
        const float minVisibleRatio = std::clamp(p.refinedMinVisibleRatio, 0.0f, 1.0f);
        const float minCorrespondenceRatio = std::clamp(
            p.refinedMinCorrespondenceRatio, 0.0f, 1.0f);
        const int minValidCorrespondences = std::max(
            p.subpixelMinCorrespondences,
            static_cast<int>(std::ceil(model.points.size() * minCorrespondenceRatio)));
        const float maxRmsResidual = std::max(0.0f, p.refinedMaxRmsResidual);
        results.erase(std::remove_if(results.begin(), results.end(),
            [&](const MatchResultV3& result) {
                return !result.refined
                    || result.visibleRatio < minVisibleRatio
                    || result.validCorrespondences < minValidCorrespondences
                    || !std::isfinite(result.rmsResidual)
                    || result.rmsResidual > maxRmsResidual;
            }), results.end());
        std::sort(results.begin(), results.end(), [](const MatchResultV3& a, const MatchResultV3& b) {
            return a.score > b.score;
        });
        std::vector<MatchResultV3> refinedNms;
        refinedNms.reserve(static_cast<size_t>(p.maxMatches));
        const double templateDistance = std::max(0.0f, p.refinedNmsTemplateFraction)
            * static_cast<double>(std::min(model.templateSize.width, model.templateSize.height));
        const double duplicateDistance = std::max(
            static_cast<double>(p.nmsDistance), templateDistance);
        const double duplicateDistance2 = duplicateDistance * duplicateDistance;
        for (const MatchResultV3& result : results) {
            bool duplicate = false;
            for (const MatchResultV3& kept : refinedNms) {
                const double dx = result.pose.x - kept.pose.x;
                const double dy = result.pose.y - kept.pose.y;
                if (dx * dx + dy * dy <= duplicateDistance2) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) refinedNms.push_back(result);
            if (static_cast<int>(refinedNms.size()) >= p.maxMatches) break;
        }
        results = std::move(refinedNms);
        stats.refineTimeMs = ms(t0);
    } else {
        stats.refineTimeMs = 0.0;
    }
    stats.totalTimeMs = ms(totalT0);
    if (outputStats) *outputStats = stats;
    return results;
}

} // namespace ShapeMatch
