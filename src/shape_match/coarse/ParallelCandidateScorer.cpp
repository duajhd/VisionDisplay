#include "shape_match/coarse/ParallelCandidateScorer.h"

#include <algorithm>
#include <chrono>
#include <future>
#include <thread>
#include <utility>

namespace ShapeMatch {

namespace {

double elapsedMsSince(std::chrono::steady_clock::time_point t0)
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

void sortAndTrim(std::vector<CoarseCandidate>& candidates, int limit)
{
    std::sort(candidates.begin(), candidates.end(), [](const CoarseCandidate& a, const CoarseCandidate& b) {
        return a.fastScore > b.fastScore;
    });
    if (limit > 0 && static_cast<int>(candidates.size()) > limit) {
        candidates.resize(static_cast<size_t>(limit));
    }
}

} // namespace

ParallelCandidateScorer::ParallelCandidateScorer(CoarseMatchConfig config)
    : m_config(std::move(config))
{
}

ParallelScoringResult ParallelCandidateScorer::scoreCandidates(const std::vector<MatchPose>& poses,
                                                               const TemplatePointSoA& points,
                                                               const RotatedTemplateCache& rotationCache,
                                                               const EdgeImageData& imageLevel,
                                                               int pyramidLevel,
                                                               int topK) const
{
    ParallelScoringResult result;
    if (poses.empty()) {
        return result;
    }

    const auto t0 = std::chrono::steady_clock::now();
    result.numThreads = threadCountFor(static_cast<int>(poses.size()));
    result.candidatesPerThread.assign(static_cast<size_t>(result.numThreads), 0);
    std::vector<std::future<std::vector<CoarseCandidate>>> futures;
    futures.reserve(static_cast<size_t>(result.numThreads));

    for (int threadIndex = 0; threadIndex < result.numThreads; ++threadIndex) {
        const int begin = static_cast<int>((static_cast<long long>(poses.size()) * threadIndex) / result.numThreads);
        const int end = static_cast<int>((static_cast<long long>(poses.size()) * (threadIndex + 1)) / result.numThreads);
        result.candidatesPerThread[static_cast<size_t>(threadIndex)] = end - begin;
        futures.push_back(std::async(std::launch::async, [this, &poses, &points, &rotationCache, &imageLevel, pyramidLevel, begin, end]() {
            FastPoseScorer scorer(m_config);
            std::vector<CoarseCandidate> local;
            local.reserve(static_cast<size_t>(std::max(0, end - begin)));
            FastScoreContext context;
            for (int i = begin; i < end; ++i) {
                CoarseCandidate candidate = scorer.scorePoseFast(points, rotationCache, imageLevel, poses[static_cast<size_t>(i)], pyramidLevel, context);
                local.push_back(std::move(candidate));
            }
            return local;
        }));
    }
    for (auto& future : futures) {
        std::vector<CoarseCandidate> local = future.get();
        result.candidates.insert(result.candidates.end(),
                                 std::make_move_iterator(local.begin()),
                                 std::make_move_iterator(local.end()));
    }
    result.scoringTimeMs = elapsedMsSince(t0);
    const auto mergeT0 = std::chrono::steady_clock::now();
    (void)topK;
    result.mergeTimeMs = elapsedMsSince(mergeT0);
    return result;
}

ParallelScoringResult ParallelCandidateScorer::scoreCandidates(const std::vector<MatchPose>& poses,
                                                               const TemplatePointSoA& points,
                                                               const RotatedTemplateCache& rotationCache,
                                                               const EdgeQueryContext& queryContext,
                                                               int pyramidLevel,
                                                               int topK) const
{
    ParallelScoringResult result;
    if (poses.empty()) {
        return result;
    }

    const auto t0 = std::chrono::steady_clock::now();
    result.numThreads = threadCountFor(static_cast<int>(poses.size()));
    result.candidatesPerThread.assign(static_cast<size_t>(result.numThreads), 0);
    std::vector<std::future<std::vector<CoarseCandidate>>> futures;
    futures.reserve(static_cast<size_t>(result.numThreads));

    for (int threadIndex = 0; threadIndex < result.numThreads; ++threadIndex) {
        const int begin = static_cast<int>((static_cast<long long>(poses.size()) * threadIndex) / result.numThreads);
        const int end = static_cast<int>((static_cast<long long>(poses.size()) * (threadIndex + 1)) / result.numThreads);
        result.candidatesPerThread[static_cast<size_t>(threadIndex)] = end - begin;
        futures.push_back(std::async(std::launch::async, [this, &poses, &points, &rotationCache, &queryContext, pyramidLevel, begin, end]() {
            FastPoseScorer scorer(m_config);
            EdgeQueryContext threadContext = queryContext;
            std::vector<CoarseCandidate> local;
            local.reserve(static_cast<size_t>(std::max(0, end - begin)));
            FastScoreContext context;
            for (int i = begin; i < end; ++i) {
                CoarseCandidate candidate = scorer.scorePoseFast(points, rotationCache, threadContext, poses[static_cast<size_t>(i)], pyramidLevel, context);
                local.push_back(std::move(candidate));
            }
            return local;
        }));
    }
    for (auto& future : futures) {
        std::vector<CoarseCandidate> local = future.get();
        result.candidates.insert(result.candidates.end(),
                                 std::make_move_iterator(local.begin()),
                                 std::make_move_iterator(local.end()));
    }
    result.scoringTimeMs = elapsedMsSince(t0);
    const auto mergeT0 = std::chrono::steady_clock::now();
    (void)topK;
    result.mergeTimeMs = elapsedMsSince(mergeT0);
    return result;
}

int ParallelCandidateScorer::threadCountFor(int candidateCount) const
{
    if (!m_config.enableParallelCandidateScoring || candidateCount < std::max(1, m_config.minCandidatesForParallelScoring)) {
        return 1;
    }
    int threads = m_config.numScoringThreads;
    if (threads <= 0) {
        threads = static_cast<int>(std::thread::hardware_concurrency());
    }
    threads = std::clamp(threads, 1, 32);
    return std::min(threads, std::max(1, candidateCount));
}

} // namespace ShapeMatch
