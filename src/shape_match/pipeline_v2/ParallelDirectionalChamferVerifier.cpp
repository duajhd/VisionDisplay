#include "shape_match/pipeline_v2/ParallelDirectionalChamferVerifier.h"

#include <algorithm>
#include <future>
#include <thread>

namespace ShapeMatch {

namespace {

int resolveThreadCount(const PipelineV2ProductionConfig& config, int workCount)
{
    if (!config.enableParallelChamferVerification || workCount <= 1) {
        return 1;
    }
    int threads = config.numWorkerThreads;
    if (threads <= 0) {
        threads = static_cast<int>(std::thread::hardware_concurrency());
    }
    return std::max(1, std::min(threads, workCount));
}

} // namespace

ParallelDirectionalChamferVerifier::ParallelDirectionalChamferVerifier(const PipelineV2ProductionConfig& productionConfig)
    : m_config(productionConfig)
{
}

std::vector<VerifiedResponsePeak> ParallelDirectionalChamferVerifier::verify(const std::vector<VerifiedResponsePeak>& input,
                                                                             const SegmentTemplate& templ,
                                                                             const DirectionalDistanceField& field,
                                                                             const DirectionalChamferVerifier& verifier,
                                                                             int targetOutputCount,
                                                                             int* prunedCount) const
{
    if (prunedCount != nullptr) {
        *prunedCount = 0;
    }
    const int count = static_cast<int>(input.size());
    const int threads = resolveThreadCount(m_config, count);
    auto runRange = [&](int begin, int end) {
        std::vector<VerifiedResponsePeak> local;
        int localPruned = 0;
        for (int i = begin; i < end; ++i) {
            VerifiedResponsePeak peak = input[static_cast<size_t>(i)];
            DirectionalChamferScore score = verifier.scoreCandidate(peak.pose, templ, field);
            peak.combinedScore = score.finalScore;
            peak.accepted = score.accepted;
            peak.rejectReason = score.rejectReason;
            if (score.pruned) {
                ++localPruned;
            }
            if (score.accepted) {
                local.push_back(peak);
            }
        }
        return std::make_pair(local, localPruned);
    };

    std::vector<VerifiedResponsePeak> merged;
    int totalPruned = 0;
    if (threads == 1) {
        auto result = runRange(0, count);
        merged = std::move(result.first);
        totalPruned = result.second;
    } else {
        std::vector<std::future<std::pair<std::vector<VerifiedResponsePeak>, int>>> futures;
        const int chunk = (count + threads - 1) / threads;
        for (int t = 0; t < threads; ++t) {
            const int begin = t * chunk;
            const int end = std::min(count, begin + chunk);
            if (begin < end) {
                futures.push_back(std::async(std::launch::async, runRange, begin, end));
            }
        }
        for (auto& f : futures) {
            auto result = f.get();
            totalPruned += result.second;
            merged.insert(merged.end(), result.first.begin(), result.first.end());
        }
    }

    std::sort(merged.begin(), merged.end(), [](const VerifiedResponsePeak& a, const VerifiedResponsePeak& b) {
        return a.combinedScore > b.combinedScore;
    });
    if (static_cast<int>(merged.size()) > targetOutputCount) {
        merged.resize(static_cast<size_t>(targetOutputCount));
    }
    if (prunedCount != nullptr) {
        *prunedCount = totalPruned;
    }
    return merged;
}

} // namespace ShapeMatch
