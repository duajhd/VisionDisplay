#include "shape_match/evaluation/CandidateRanker.h"

#include "shape_match/evaluation/FinalCandidateNms.h"

#include <algorithm>
#include <chrono>
#include <limits>
#include <thread>
#include <vector>

namespace ShapeMatch {

namespace {

double elapsedMsSince(std::chrono::steady_clock::time_point t0)
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

int resolveThreadCount(const ShapeMatchEvalConfig& config, int count)
{
    if (!config.enableParallelFinalRanking || count < 2) {
        return 1;
    }
    int threads = config.finalRankerNumThreads;
    if (threads <= 0) {
        threads = static_cast<int>(std::thread::hardware_concurrency());
    }
    threads = std::max(1, threads);
    return std::min(threads, count);
}

void evaluateGroundTruthFields(ScoredCandidate& candidate,
                               const ShapeTemplateModel& model,
                               const std::vector<GroundTruthInstance>* groundTruth,
                               const PoseErrorEvaluator& poseEvaluator)
{
    if (!groundTruth || groundTruth->empty()) {
        return;
    }
    PoseError bestError;
    ContourReprojectionError bestContour;
    double bestCost = std::numeric_limits<double>::max();
    for (const GroundTruthInstance& gt : *groundTruth) {
        PoseError poseError = poseEvaluator.evaluatePoseError(candidate.pose, gt.pose);
        const double cost = poseError.dxy + 0.5 * poseError.dthetaDeg + std::abs(poseError.dscale) * 10.0;
        if (cost < bestCost) {
            bestCost = cost;
            bestError = poseError;
            bestContour = poseEvaluator.evaluateContourReprojectionError(model, candidate.pose, gt.pose);
        }
    }
    candidate.poseError = bestError;
    candidate.contourError = bestContour;
}

} // namespace

CandidateRanker::CandidateRanker(ShapeMatchEvalConfig config)
    : m_config(config)
{
}

std::vector<ScoredCandidate> CandidateRanker::rank(const std::vector<MatchPose>& candidates,
                                                   const ShapeTemplateModel& model,
                                                   const EdgeImageData& edgeData,
                                                   const std::vector<GroundTruthInstance>* groundTruth,
                                                   FinalRankerProfile* profile) const
{
    EdgeQueryContext context;
    context.fullEdgeData = &edgeData;
    context.roiFields = nullptr;
    context.preferRoiField = false;
    context.allowFullField = true;
    context.allowLocalWindowFallback = m_config.allowLocalSearchFallbackInEvaluator;
    return rank(candidates, model, context, groundTruth, profile);
}

std::vector<ScoredCandidate> CandidateRanker::rank(const std::vector<MatchPose>& candidates,
                                                   const ShapeTemplateModel& model,
                                                   const EdgeQueryContext& queryContext,
                                                   const std::vector<GroundTruthInstance>* groundTruth,
                                                   FinalRankerProfile* profile) const
{
    const auto totalT0 = std::chrono::steady_clock::now();
    FinalRankerProfile localProfile;
    FinalRankerProfile& p = profile ? *profile : localProfile;
    p = FinalRankerProfile{};
    p.candidateCountBeforeNms = static_cast<int>(candidates.size());
    p.distanceFieldValid = queryContext.fullEdgeData && queryContext.fullEdgeData->hasNearestEdgeField();

    FinalCandidateNms nms(m_config);
    const FinalCandidateNmsResult nmsResult = nms.apply(candidates);
    p.candidateCountAfterNms = nmsResult.afterNmsCount;
    p.candidateCountActuallyEvaluated = nmsResult.afterNmsCount;

    const int count = static_cast<int>(nmsResult.poses.size());
    const int numThreads = resolveThreadCount(m_config, count);
    p.numThreads = numThreads;
    std::vector<ScoredCandidate> scored(static_cast<size_t>(count));
    std::vector<MatchEvaluationStats> stats(static_cast<size_t>(count));

    const auto scoreT0 = std::chrono::steady_clock::now();
    auto evaluateRange = [&](int begin, int end) {
        MatchEvaluator evaluator(m_config);
        PoseErrorEvaluator poseEvaluator(m_config);
        EdgeQueryContext threadContext = queryContext;
        for (int i = begin; i < end; ++i) {
            ScoredCandidate candidate;
            candidate.pose = nmsResult.poses[static_cast<size_t>(i)];
            candidate.score = evaluator.evaluate(model,
                                                 threadContext,
                                                 candidate.pose,
                                                 nullptr,
                                                 &stats[static_cast<size_t>(i)]);
            evaluateGroundTruthFields(candidate, model, groundTruth, poseEvaluator);
            scored[static_cast<size_t>(i)] = std::move(candidate);
        }
    };

    if (numThreads <= 1) {
        evaluateRange(0, count);
    } else {
        std::vector<std::thread> workers;
        workers.reserve(static_cast<size_t>(numThreads));
        int begin = 0;
        for (int t = 0; t < numThreads; ++t) {
            const int remaining = count - begin;
            const int remainingThreads = numThreads - t;
            const int span = (remaining + remainingThreads - 1) / remainingThreads;
            const int end = std::min(count, begin + span);
            workers.emplace_back(evaluateRange, begin, end);
            begin = end;
        }
        for (std::thread& worker : workers) {
            worker.join();
        }
    }
    p.scoreOnlyTimeMs = elapsedMsSince(scoreT0);

    for (const MatchEvaluationStats& s : stats) {
        p.linearScanFallbackCount += s.linearScanFallbackCount;
        p.localWindowFallbackCount += s.localWindowFallbackCount;
        p.roiDistanceFieldQueryCount += s.roiDistanceFieldQueryCount;
        p.fullDistanceFieldQueryCount += s.fullDistanceFieldQueryCount;
        p.missingFieldCount += s.missingFieldCount;
        if (s.evaluationMode == "roi_distance_field") {
            p.evaluationMode = "roi_distance_field";
        } else if (s.evaluationMode == "linear_scan_debug_only") {
            p.evaluationMode = "linear_scan_debug_only";
        } else if (s.evaluationMode == "local_window" && p.evaluationMode != "linear_scan_debug_only") {
            p.evaluationMode = "local_window";
        } else if (p.evaluationMode == "unknown" && (s.evaluationMode == "distance_field" || s.evaluationMode == "full_distance_field")) {
            p.evaluationMode = "full_distance_field";
        }
    }
    if (p.evaluationMode == "unknown") {
        p.evaluationMode = queryContext.roiFields ? "roi_distance_field" : (p.distanceFieldValid ? "full_distance_field" : "local_window");
    }

    std::sort(scored.begin(), scored.end(), [](const ScoredCandidate& a, const ScoredCandidate& b) {
        return a.score.finalScore > b.score.finalScore;
    });

    if (m_config.topK > 0 && static_cast<int>(scored.size()) > m_config.topK) {
        scored.resize(static_cast<size_t>(m_config.topK));
    }
    for (size_t i = 0; i < scored.size(); ++i) {
        scored[i].rank = static_cast<int>(i) + 1;
    }

    const bool writeAllPointEvals = m_config.enablePointEvaluations;
    const bool writeTopPointEvals = !writeAllPointEvals && m_config.enablePointEvaluationsForTopKOnly;
    const int pointEvalCount = writeAllPointEvals
        ? static_cast<int>(scored.size())
        : (writeTopPointEvals ? std::min<int>(m_config.pointEvaluationTopK, static_cast<int>(scored.size())) : 0);
    p.pointEvalEnabled = pointEvalCount > 0;
    p.pointEvalTopK = pointEvalCount;
    if (pointEvalCount > 0) {
        const auto pointEvalT0 = std::chrono::steady_clock::now();
        MatchEvaluator evaluator(m_config);
        EdgeQueryContext pointEvalContext = queryContext;
        for (int i = 0; i < pointEvalCount; ++i) {
            scored[static_cast<size_t>(i)].score = evaluator.evaluate(model,
                                                                      pointEvalContext,
                                                                      scored[static_cast<size_t>(i)].pose,
                                                                      &scored[static_cast<size_t>(i)].pointEvaluations,
                                                                      nullptr);
        }
        p.pointEvalTimeMs = elapsedMsSince(pointEvalT0);
    }
    p.totalFinalRankerTimeMs = elapsedMsSince(totalT0);
    p.avgTimePerCandidateMs = p.candidateCountActuallyEvaluated > 0
        ? p.scoreOnlyTimeMs / static_cast<double>(p.candidateCountActuallyEvaluated)
        : 0.0;
    const long long totalTemplatePoints = static_cast<long long>(std::max(1, p.candidateCountActuallyEvaluated))
        * static_cast<long long>(std::max<size_t>(1, model.points.size()));
    p.avgTimePerTemplatePointUs = totalTemplatePoints > 0
        ? p.scoreOnlyTimeMs * 1000.0 / static_cast<double>(totalTemplatePoints)
        : 0.0;
    return scored;
}

} // namespace ShapeMatch
