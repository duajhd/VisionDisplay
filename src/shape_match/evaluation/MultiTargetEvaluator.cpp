#include "shape_match/evaluation/MultiTargetEvaluator.h"

#include <algorithm>

namespace ShapeMatch {

namespace {

struct PairMatch
{
    int candidateIndex = -1;
    int gtIndex = -1;
    double cost = 0.0;
};

} // namespace

MultiTargetEvaluator::MultiTargetEvaluator(ShapeMatchEvalConfig config)
    : m_config(config)
{
}

bool MultiTargetEvaluator::candidateHitsAnyGt(const ScoredCandidate& candidate,
                                              const std::vector<GroundTruthInstance>& groundTruth) const
{
    PoseErrorEvaluator evaluator(m_config);
    for (const GroundTruthInstance& gt : groundTruth) {
        const PoseError e = evaluator.evaluatePoseError(candidate.pose, gt.pose);
        if (e.xyOk && e.angleOk) {
            return true;
        }
    }
    return false;
}

MultiTargetEvalResult MultiTargetEvaluator::evaluate(const std::vector<ScoredCandidate>& candidates,
                                                     const std::vector<GroundTruthInstance>& groundTruth) const
{
    MultiTargetEvalResult result;
    result.candidateCount = static_cast<int>(candidates.size());
    result.gtCount = static_cast<int>(groundTruth.size());

    PoseErrorEvaluator evaluator(m_config);
    std::vector<PairMatch> pairs;
    for (size_t ci = 0; ci < candidates.size(); ++ci) {
        for (size_t gi = 0; gi < groundTruth.size(); ++gi) {
            const PoseError e = evaluator.evaluatePoseError(candidates[ci].pose, groundTruth[gi].pose);
            if (e.xyOk && e.angleOk) {
                pairs.push_back(PairMatch{static_cast<int>(ci),
                                          static_cast<int>(gi),
                                          e.dxy + m_config.multiTargetAngleCostWeight * e.dthetaDeg});
            }
        }
    }

    std::sort(pairs.begin(), pairs.end(), [](const PairMatch& a, const PairMatch& b) {
        return a.cost < b.cost;
    });

    std::vector<bool> candidateMatched(candidates.size(), false);
    std::vector<bool> gtMatched(groundTruth.size(), false);
    for (const PairMatch& pair : pairs) {
        if (candidateMatched[static_cast<size_t>(pair.candidateIndex)] || gtMatched[static_cast<size_t>(pair.gtIndex)]) {
            continue;
        }
        candidateMatched[static_cast<size_t>(pair.candidateIndex)] = true;
        gtMatched[static_cast<size_t>(pair.gtIndex)] = true;
        result.matchedCandidateIndices.push_back(pair.candidateIndex);
        result.matchedGtIndices.push_back(pair.gtIndex);
    }

    result.truePositive = static_cast<int>(result.matchedCandidateIndices.size());
    result.falsePositive = result.candidateCount - result.truePositive;
    result.falseNegative = result.gtCount - result.truePositive;

    for (size_t ci = 0; ci < candidates.size(); ++ci) {
        if (candidateMatched[ci]) {
            continue;
        }
        if (candidateHitsAnyGt(candidates[ci], groundTruth)) {
            ++result.duplicateCount;
        }
    }

    const double pDen = static_cast<double>(result.truePositive + result.falsePositive);
    const double rDen = static_cast<double>(result.truePositive + result.falseNegative);
    result.precision = pDen > 0.0 ? static_cast<double>(result.truePositive) / pDen : 0.0;
    result.recall = rDen > 0.0 ? static_cast<double>(result.truePositive) / rDen : 0.0;
    result.f1 = (result.precision + result.recall) > 0.0
        ? 2.0 * result.precision * result.recall / (result.precision + result.recall)
        : 0.0;

    auto hitWithin = [&](int k) {
        const int n = std::min(k, static_cast<int>(candidates.size()));
        for (int i = 0; i < n; ++i) {
            if (candidateHitsAnyGt(candidates[static_cast<size_t>(i)], groundTruth)) {
                return true;
            }
        }
        return false;
    };
    result.top1Hit = hitWithin(1);
    result.top3Hit = hitWithin(3);
    result.top5Hit = hitWithin(5);
    result.top10Hit = hitWithin(10);
    return result;
}

} // namespace ShapeMatch
