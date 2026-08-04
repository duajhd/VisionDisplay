#include "shape_match/pipeline_v3/AngleViewBuilderV3.h"
#include "shape_match/pipeline_v3/WorkerPoolV3.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <tuple>

namespace ShapeMatch {
namespace {

std::vector<int> stageLimits(int count, const ShapeSearchParametersV3& p)
{
    std::vector<int> limits;
    for (int v : {p.stage0PointCount, p.stage1PointCount, p.stage2PointCount, count}) {
        v = std::clamp(v, 1, count);
        if (limits.empty() || v > limits.back()) limits.push_back(v);
    }
    return limits;
}

float greedyThreshold(SearchSafetyV3 safety, int stage, const ShapeSearchParametersV3& p)
{
    if (safety == SearchSafetyV3::Safe || stage >= 3) return 0.0f;
    return safety == SearchSafetyV3::Balanced ? p.balancedGreedyMean[static_cast<size_t>(stage)]
                                              : p.fastGreedyMean[static_cast<size_t>(stage)];
}

} // namespace

std::vector<AngleViewV3> AngleViewBuilderV3::build(const ShapeModelV3& model, float scale,
                                                   int stride, float angleStart,
                                                   float angleExtent, float angleStep,
                                                   const ShapeSearchParametersV3& search) const
{
    if (model.empty() || stride <= 0 || scale <= 0.0f) throw std::invalid_argument("invalid V3 angle-view input");
    if (angleStep <= 0.0f) angleStep = static_cast<float>(2.0 * kPi);
    const int angleCount = angleExtent <= 0.0f ? 1
        : std::max(1, static_cast<int>(std::floor(angleExtent / angleStep + 0.5f)) + 1);
    std::vector<AngleViewV3> result(static_cast<size_t>(angleCount));
    auto buildAngle = [&](int ai) {
        AngleViewV3 view;
        view.angleRadians = angleStart + ai * angleStep;
        const float c = std::cos(view.angleRadians);
        const float s = std::sin(view.angleRadians);

        // Preserve quality order while merging points that quantize to the same rotated pixel.
        using PointKey = std::tuple<int, int, int>;
        struct MergeValue { int rank = 0; int weight = 0; std::uint16_t partId = 0; };
        std::map<PointKey, MergeValue> merged;
        int rank = 0;
        for (const ShapePointV3& p : model.points) {
            const int dx = static_cast<int>(std::lround(scale * (p.x * c - p.y * s)));
            const int dy = static_cast<int>(std::lround(scale * (p.x * s + p.y * c)));
            const int bin = 0;
            auto [it, inserted] = merged.emplace(PointKey(dx, dy, bin), MergeValue{rank, 0, p.partId});
            it->second.weight = std::min(255, it->second.weight + std::max(1, static_cast<int>(p.weight)));
            if (inserted) ++rank;
        }
        struct Ranked { int dx; int dy; int bin; int rank; int weight; std::uint16_t partId; };
        std::vector<Ranked> ranked;
        ranked.reserve(merged.size());
        for (const auto& [key, value] : merged)
            ranked.push_back({std::get<0>(key), std::get<1>(key), std::get<2>(key),
                              value.rank, value.weight, value.partId});
        std::stable_sort(ranked.begin(), ranked.end(), [](const Ranked& a, const Ranked& b) {
            return a.rank < b.rank;
        });
        if (ranked.empty()) return;
        const std::vector<int> limits = stageLimits(static_cast<int>(ranked.size()), search);
        int begin = 0;
        std::uint32_t cumulative = 0;
        view.minDx = view.minDy = std::numeric_limits<int>::max();
        view.maxDx = view.maxDy = std::numeric_limits<int>::min();
        for (size_t stageIndex = 0; stageIndex < limits.size(); ++stageIndex) {
            const int end = limits[stageIndex];
            std::vector<Ranked> slice(ranked.begin() + begin, ranked.begin() + end);
            std::sort(slice.begin(), slice.end(), [](const Ranked& a, const Ranked& b) {
                return a.dy != b.dy ? a.dy < b.dy : a.dx < b.dx;
            });
            for (const Ranked& r : slice) {
                if (r.dx < std::numeric_limits<std::int16_t>::min() || r.dx > std::numeric_limits<std::int16_t>::max()
                    || r.dy < std::numeric_limits<std::int16_t>::min() || r.dy > std::numeric_limits<std::int16_t>::max())
                    throw std::overflow_error("V3 rotated template exceeds int16 offsets");
                RotatedPointV3 point;
                point.dx = static_cast<std::int16_t>(r.dx);
                point.dy = static_cast<std::int16_t>(r.dy);
                const std::int64_t linear = static_cast<std::int64_t>(r.dy) * stride + r.dx;
                if (linear < INT32_MIN || linear > INT32_MAX) throw std::overflow_error("V3 linear offset overflow");
                point.linearOffset = static_cast<std::int32_t>(linear);
                point.orientationBin = static_cast<std::uint8_t>(r.bin);
                point.weight = static_cast<std::uint8_t>(r.weight);
                point.partId = r.partId;
                view.points.push_back(point);
                if (cumulative > std::numeric_limits<std::uint32_t>::max() - point.weight)
                    throw std::overflow_error("V3 weight sum exceeds uint32 accumulator");
                cumulative += point.weight;
                view.minDx = std::min(view.minDx, r.dx); view.maxDx = std::max(view.maxDx, r.dx);
                view.minDy = std::min(view.minDy, r.dy); view.maxDy = std::max(view.maxDy, r.dy);
            }
            ScoreStageV3 stage;
            stage.begin = static_cast<std::uint32_t>(begin);
            stage.end = static_cast<std::uint32_t>(end);
            stage.cumulativeWeight = cumulative;
            stage.greedyMaxMeanDistance = greedyThreshold(search.safetyMode, static_cast<int>(stageIndex), search);
            view.stages.push_back(stage);
            begin = end;
        }
        view.totalWeight = cumulative;
        if (view.totalWeight > static_cast<std::uint32_t>(INT32_MAX / 255))
            throw std::overflow_error("V3 weighted distance cost exceeds signed AVX2 range");
        for (ScoreStageV3& stage : view.stages) {
            stage.remainingWeight = view.totalWeight - stage.cumulativeWeight;
        }
        result[static_cast<size_t>(ai)] = std::move(view);
    };

    const int threadCount = search.enableMultithreading
        ? std::min(WorkerPoolV3::shared().threadCount(), angleCount) : 1;
    if (threadCount == 1) {
        for (int ai = 0; ai < angleCount; ++ai) buildAngle(ai);
    } else {
        WorkerPoolV3::shared().parallelFor(0, angleCount, threadCount,
            [&](int begin, int end, int) {
                for (int ai = begin; ai < end; ++ai) buildAngle(ai);
            });
    }
    return result;
}

void AngleViewBuilderV3::precompute(ShapeModelV3& model) const
{
    if (model.empty()) throw std::invalid_argument("cannot precompute an empty V3 model");
    ShapeSearchParametersV3 parameters;
    parameters.enableMultithreading = true;
    parameters.numThreads = 4;
    parameters.stage0PointCount = model.parameters.stage0PointCount;
    parameters.stage1PointCount = model.parameters.stage1PointCount;
    parameters.stage2PointCount = model.parameters.stage2PointCount;
    const float step = static_cast<float>(2.0 * kPi / kPrecomputedAngleCountV3);
    const float start = static_cast<float>(-kPi);
    const float extent = step * static_cast<float>(kPrecomputedAngleCountV3 - 1);
    for (auto& views : model.precomputedAngleViews) views.clear();
    model.precomputedAngleViews[3] = build(model, 0.125f, 1, start, extent, step, parameters);
}

std::vector<AngleViewV3> AngleViewBuilderV3::bindPrecomputed(
    const ShapeModelV3& model, int pyramidLevel, int stride) const
{
    if (!model.hasPrecomputedViews())
        throw std::invalid_argument("V3 model has no precomputed angle views");
    if (pyramidLevel != 3 || stride <= 0)
        throw std::invalid_argument("V3 only supports precomputed Level 3 angle views");
    std::vector<AngleViewV3> result =
        model.precomputedAngleViews[static_cast<size_t>(pyramidLevel)];
    for (AngleViewV3& view : result) {
        for (RotatedPointV3& point : view.points) {
            const std::int64_t offset = static_cast<std::int64_t>(point.dy) * stride + point.dx;
            if (offset < INT32_MIN || offset > INT32_MAX)
                throw std::overflow_error("V3 precomputed linear offset overflow");
            point.linearOffset = static_cast<std::int32_t>(offset);
        }
    }
    return result;
}

} // namespace ShapeMatch
