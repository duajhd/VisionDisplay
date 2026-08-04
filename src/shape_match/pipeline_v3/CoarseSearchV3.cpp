#include "shape_match/pipeline_v3/CoarseSearchV3.h"

#include "shape_match/pipeline_v3/ScoreKernelV3.h"
#include "shape_match/pipeline_v3/WorkerPoolV3.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ShapeMatch {
namespace {

struct Bounds { int left = 0; int top = 0; int right = 0; int bottom = 0; };

Bounds validBounds(const ResponseMapV3& r, const AngleViewV3& v, cv::Rect roi)
{
    const cv::Rect image(0, 0, r.width, r.height);
    if (roi.width <= 0 || roi.height <= 0) roi = image;
    roi &= image;
    Bounds b;
    b.left = std::max(roi.x, -v.minDx);
    b.right = std::min(roi.x + roi.width, r.width - v.maxDx);
    b.top = std::max(roi.y, -v.minDy);
    b.bottom = std::min(roi.y + roi.height, r.height - v.maxDy);
    return b;
}

std::uint32_t rawThreshold(float minScore, std::uint32_t totalPointCount)
{
    minScore = std::clamp(minScore, 0.0f, 1.0f);
    return static_cast<std::uint32_t>(std::floor((1.0f - minScore) * 255.0f * totalPointCount));
}

std::uint32_t scoreOne(const ResponseMapV3& response, int base, const AngleViewV3& view,
                       std::uint32_t, bool, std::uint8_t* rejectedStage)
{
    std::uint32_t sum = 0;
    for (const RotatedPointV3& point : view.points)
        sum += static_cast<std::uint32_t>(response.binData[point.orientationBin]
            [base + point.linearOffset]) * point.weight;
    if (rejectedStage) *rejectedStage = 0;
    return sum;
}

void countReject(ShapeMatchStatisticsV3& s, std::uint8_t stage)
{
    if (stage == 1) ++s.rejectedAtStage0;
    else if (stage == 2) ++s.rejectedAtStage1;
    else if (stage == 3) ++s.rejectedAtStage2;
    else if (stage == 0) ++s.fullyEvaluated;
}

struct ThreadResult { std::vector<MatchCandidateV3> candidates; ShapeMatchStatisticsV3 stats; };

} // namespace

void CandidateCollectorV3::keepTopK(std::vector<MatchCandidateV3>& candidates, int capacity)
{
    if (capacity <= 0) { candidates.clear(); return; }
    auto better = [](const MatchCandidateV3& a, const MatchCandidateV3& b) {
        return a.rawCost < b.rawCost;
    };
    if (static_cast<int>(candidates.size()) > capacity) {
        std::nth_element(candidates.begin(), candidates.begin() + capacity, candidates.end(), better);
        candidates.resize(static_cast<size_t>(capacity));
    }
    std::sort(candidates.begin(), candidates.end(), better);
}

std::vector<MatchCandidateV3> PoseNmsV3::apply(const std::vector<MatchCandidateV3>& input,
                                               const std::vector<AngleViewV3>& views,
                                               int maxCount, float dp, float da) const
{
    std::vector<MatchCandidateV3> out;
    out.reserve(static_cast<size_t>(std::max(0, maxCount)));
    const float dp2 = dp * dp;
    for (const MatchCandidateV3& c : input) {
        bool duplicate = false;
        for (const MatchCandidateV3& k : out) {
            const float dx = static_cast<float>(c.x - k.x), dy = static_cast<float>(c.y - k.y);
            float angle = std::abs(views[static_cast<size_t>(c.angleIndex)].angleRadians
                                   - views[static_cast<size_t>(k.angleIndex)].angleRadians);
            angle = std::min(angle, static_cast<float>(2.0 * kPi) - angle);
            if (dx * dx + dy * dy < dp2 && angle < da) { duplicate = true; break; }
        }
        if (!duplicate) out.push_back(c);
        if (static_cast<int>(out.size()) >= maxCount) break;
    }
    return out;
}

std::vector<MatchCandidateV3> CoarseSearchV3::search(const ResponseMapV3& response,
                                                     const PyramidLevelModelV3& model,
                                                     const ShapeSearchParametersV3& p,
                                                     ShapeMatchStatisticsV3* statistics,
                                                     WorkerPoolV3* workerPool) const
{
    if (!response.valid() || model.angleViews.empty()) return {};
    int threads = p.enableMultithreading && workerPool
        ? std::min(p.numThreads, workerPool->threadCount()) : 1;
    threads = std::clamp(threads, 1, 4);
    std::vector<ThreadResult> locals(static_cast<size_t>(threads));
    const bool useAvx512 = p.enableAvx2 && cpuSupportsAvx512V3();
    const bool useAvx2 = p.enableAvx2 && cpuSupportsAvx2V3();
    const bool greedy = p.safetyMode != SearchSafetyV3::Safe;
    auto runWorker = [&](int ti) {
            ThreadResult& local = locals[static_cast<size_t>(ti)];
            local.candidates.reserve(static_cast<size_t>(std::max(16, p.coarseTopK * 2)));
            for (int angle = 0; angle < static_cast<int>(model.angleViews.size()); ++angle) {
                const AngleViewV3& view = model.angleViews[static_cast<size_t>(angle)];
                const Bounds b = validBounds(response, view, p.searchRoi);
                const int blockTop = b.top + (b.bottom - b.top) * ti / threads;
                const int blockBottom = b.top + (b.bottom - b.top) * (ti + 1) / threads;
                std::uint32_t threshold = rawThreshold(p.minScore, view.totalWeight);
                for (int y = blockTop; y < blockBottom; ++y) {
                    int x = b.left;
                    if (useAvx512) {
                        for (; x + 64 <= b.right; x += 64) {
                            const int base = y * response.stride + x;
                            const ScoreBlock64V3 block = scoreBlock64AVX512(
                                response, base, view, threshold);
                            ++local.stats.evaluatedBlocks;
                            ++local.stats.avx512Blocks;
                            local.stats.evaluatedCandidates += 64;
                            for (int lane = 0; lane < 64; ++lane) {
                                ++local.stats.fullyEvaluated;
                                if ((block.validMask & (std::uint64_t{1} << lane)) != 0) {
                                    local.candidates.push_back({x + lane, y, angle, 0,
                                        block.rawCosts[static_cast<size_t>(lane)],
                                        1.0f - static_cast<float>(block.rawCosts[static_cast<size_t>(lane)])
                                          / static_cast<float>(255u * view.totalWeight)});
                                }
                            }
                        }
                    }
                    if (useAvx2) {
                        for (; x + 32 <= b.right; x += 32) {
                            const int base = y * response.stride + x;
                            const ScoreBlock32V3 block = scoreBlock32AVX2(response, base, view, threshold);
                            ++local.stats.evaluatedBlocks;
                            ++local.stats.avx2Blocks;
                            local.stats.evaluatedCandidates += 32;
                            for (int lane = 0; lane < 32; ++lane) {
                                ++local.stats.fullyEvaluated;
                                if ((block.validMask & (std::uint32_t{1} << lane)) != 0) {
                                    local.candidates.push_back({x + lane, y, angle, 0,
                                        block.rawCosts[static_cast<size_t>(lane)],
                                        1.0f - static_cast<float>(block.rawCosts[static_cast<size_t>(lane)])
                                          / static_cast<float>(255u * view.totalWeight)});
                                }
                            }
                        }
                    }
                    for (; x + 16 <= b.right; x += 16) {
                        const int base = y * response.stride + x;
                        const ScoreBlock16V3 block = useAvx2
                            ? scoreBlock16AVX2(response, base, view, threshold, greedy)
                            : scoreBlock16Scalar(response, base, view, threshold, greedy);
                        ++local.stats.evaluatedBlocks;
                        useAvx2 ? ++local.stats.avx2Blocks : ++local.stats.scalarBlocks;
                        local.stats.evaluatedCandidates += 16;
                        for (int lane = 0; lane < 16; ++lane) {
                            countReject(local.stats, block.rejectedStage[static_cast<size_t>(lane)]);
                            if ((block.validMask & (1u << lane)) && block.rawCosts[static_cast<size_t>(lane)] <= threshold) {
                                local.candidates.push_back({x + lane, y, angle, 0,
                                    block.rawCosts[static_cast<size_t>(lane)],
                                    1.0f - static_cast<float>(block.rawCosts[static_cast<size_t>(lane)])
                                      / static_cast<float>(255u * view.totalWeight)});
                            }
                        }
                    }
                    for (; x < b.right; ++x) {
                        std::uint8_t rejected = 0;
                        const std::uint32_t raw = scoreOne(response, y * response.stride + x,
                                                           view, threshold, greedy, &rejected);
                        ++local.stats.evaluatedCandidates; countReject(local.stats, rejected);
                        if (!rejected && raw <= threshold)
                            local.candidates.push_back({x, y, angle, 0, raw,
                                1.0f - static_cast<float>(raw) / static_cast<float>(255u * view.totalWeight)});
                    }
                }
                if (static_cast<int>(local.candidates.size()) > p.coarseTopK * 4)
                {
                    CandidateCollectorV3::keepTopK(local.candidates, p.coarseTopK * 2);
                }
            }
            CandidateCollectorV3::keepTopK(local.candidates, p.coarseTopK);
    };
    if (threads == 1) runWorker(0);
    else workerPool->parallelFor(0, threads, threads,
        [&](int begin, int end, int) { for (int ti = begin; ti < end; ++ti) runWorker(ti); });
    std::vector<MatchCandidateV3> merged;
    merged.reserve(static_cast<size_t>(threads * std::max(0, p.coarseTopK)));
    for (ThreadResult& local : locals) {
        merged.insert(merged.end(), local.candidates.begin(), local.candidates.end());
        if (statistics) {
            statistics->evaluatedBlocks += local.stats.evaluatedBlocks;
            statistics->evaluatedCandidates += local.stats.evaluatedCandidates;
            statistics->rejectedAtStage0 += local.stats.rejectedAtStage0;
            statistics->rejectedAtStage1 += local.stats.rejectedAtStage1;
            statistics->rejectedAtStage2 += local.stats.rejectedAtStage2;
            statistics->fullyEvaluated += local.stats.fullyEvaluated;
            statistics->scalarBlocks += local.stats.scalarBlocks;
            statistics->avx2Blocks += local.stats.avx2Blocks;
            statistics->avx512Blocks += local.stats.avx512Blocks;
        }
    }
    CandidateCollectorV3::keepTopK(merged, p.coarseTopK);
    return merged;
}

} // namespace ShapeMatch
