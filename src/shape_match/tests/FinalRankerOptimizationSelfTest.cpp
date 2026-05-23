#include "shape_match/coarse/CoarseMatchConfig.h"
#include "shape_match/coarse/DistanceFieldBuilder.h"
#include "shape_match/core/EdgeImageData.h"
#include "shape_match/evaluation/CandidateRanker.h"
#include "shape_match/evaluation/FinalCandidateNms.h"
#include "shape_match/evaluation/MatchEvaluator.h"

#include <chrono>
#include <cmath>
#include <iostream>

namespace ShapeMatch {
namespace {

bool expect(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "[FinalRankerOptimization][FAIL] " << message << '\n';
        return false;
    }
    return true;
}

ShapeTemplateModel makeLineTemplate(int pointCount = 80)
{
    ShapeTemplateModel model;
    model.points.reserve(static_cast<size_t>(pointCount));
    for (int i = 0; i < pointCount; ++i) {
        TemplatePoint p;
        p.id = i;
        p.position = cv::Point2d(0.0, static_cast<double>(i - pointCount / 2));
        p.normal = cv::Point2d(1.0, 0.0);
        p.gradientDir = p.normal;
        p.tangent = cv::Point2d(0.0, 1.0);
        p.gradientMag = 255.0;
        model.points.push_back(p);
    }
    return model;
}

EdgeImageData makeVerticalLineEdgeData(cv::Size size, int x, const ShapeTemplateModel& model, const MatchPose& pose)
{
    EdgeImageData data = EdgeImageData::createFromTemplateAndPose(model, pose, size);
    CoarseMatchConfig config;
    config.enableDistanceFieldScoring = true;
    DistanceFieldBuilder builder;
    builder.build(data, config);
    (void)x;
    return data;
}

double elapsedMsSince(std::chrono::steady_clock::time_point t0)
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

bool testFastNearestEdge()
{
    const ShapeTemplateModel model = makeLineTemplate(31);
    const MatchPose pose = MatchPose::fromDeg(25.0, 25.0, 0.0);
    EdgeImageData edgeData = makeVerticalLineEdgeData(cv::Size(80, 80), 25, model, pose);

    cv::Point2d matched;
    cv::Point2d normal;
    double mag = 0.0;
    double dist = 0.0;
    const bool found = edgeData.findNearestEdgeFast(cv::Point2d(30.0, 25.0), 10.0, matched, normal, mag, dist);
    bool ok = true;
    ok &= expect(found, "fast nearest edge should find vertical line");
    ok &= expect(std::abs(matched.x - 25.0) <= 1.0, "nearest edge x should be near line x");
    ok &= expect(std::abs(dist - 5.0) <= 1.5, "distance should be near 5 px");
    ok &= expect(edgeData.hasNearestEdgeField(), "distance/nearest field should be valid");
    return ok;
}

bool testEvaluatorTrend()
{
    const ShapeTemplateModel model = makeLineTemplate(80);
    const MatchPose gt = MatchPose::fromDeg(100.0, 100.0, 0.0);
    EdgeImageData edgeData = makeVerticalLineEdgeData(cv::Size(220, 220), 100, model, gt);

    ShapeMatchEvalConfig fastConfig;
    fastConfig.searchRadiusPx = 20.0;
    fastConfig.evaluatorLocalSearchRadiusPx = 20;
    fastConfig.minValidPoints = 1;
    MatchEvaluator fastEval(fastConfig);

    ShapeMatchEvalConfig localConfig = fastConfig;
    localConfig.enableDistanceFieldEvaluation = false;
    localConfig.enableNearestEdgeFieldEvaluation = false;
    MatchEvaluator localEval(localConfig);

    const std::vector<MatchPose> poses = {
        MatchPose::fromDeg(100.0, 100.0, 0.0),
        MatchPose::fromDeg(102.0, 100.0, 0.0),
        MatchPose::fromDeg(105.0, 100.0, 0.0),
        MatchPose::fromDeg(110.0, 100.0, 0.0),
        MatchPose::fromDeg(100.0, 100.0, 10.0)
    };

    std::vector<double> fastScores;
    std::vector<double> localScores;
    MatchEvaluationStats stats;
    for (const MatchPose& pose : poses) {
        stats = {};
        fastScores.push_back(fastEval.evaluate(model, edgeData, pose, nullptr, &stats).finalScore);
        localScores.push_back(localEval.evaluate(model, edgeData, pose).finalScore);
        if (!expect(stats.linearScanFallbackCount == 0, "distance-field evaluator should not linear scan")) {
            return false;
        }
    }

    bool ok = true;
    ok &= expect(fastScores[0] >= fastScores[1], "fast GT score should be >= 2px score");
    ok &= expect(fastScores[1] >= fastScores[2], "fast 2px score should be >= 5px score");
    ok &= expect(fastScores[2] >= fastScores[3], "fast 5px score should be >= 10px score");
    ok &= expect(localScores[0] >= localScores[1], "local GT score should be >= 2px score");
    ok &= expect(localScores[1] >= localScores[2], "local 2px score should be >= 5px score");
    return ok;
}

bool testFinalCandidateNms()
{
    ShapeMatchEvalConfig config;
    config.finalNmsTranslationPx = 5.0;
    config.finalNmsAngleDeg = 2.0;
    config.targetFinalRankerInputCandidates = 60;
    config.maxFinalRankerInputCandidates = 80;
    FinalCandidateNms nms(config);

    std::vector<MatchPose> poses;
    for (int i = 0; i < 100; ++i) {
        poses.push_back(MatchPose::fromDeg(50.0 + 0.1 * (i % 4), 50.0 + 0.1 * (i % 4), 0.1 * (i % 4)));
    }
    for (int i = 0; i < 60; ++i) {
        poses.push_back(MatchPose::fromDeg(100.0 + i * 12.0, 100.0, 0.0));
    }

    const FinalCandidateNmsResult result = nms.apply(poses);
    bool ok = true;
    ok &= expect(result.inputCount == 160, "NMS input count should be 160");
    ok &= expect(result.afterNmsCount >= 30 && result.afterNmsCount <= 60, "NMS should reduce to 30..60 candidates");
    ok &= expect(!result.poses.empty() && std::abs(result.poses.front().x - 50.0) < 1.0, "top duplicate target should remain");
    return ok;
}

bool testParallelRankingConsistency()
{
    const ShapeTemplateModel model = makeLineTemplate(80);
    const MatchPose gt = MatchPose::fromDeg(120.0, 120.0, 0.0);
    EdgeImageData edgeData = makeVerticalLineEdgeData(cv::Size(260, 260), 120, model, gt);

    std::vector<MatchPose> candidates;
    for (int i = -10; i <= 10; ++i) {
        candidates.push_back(MatchPose::fromDeg(120.0 + i, 120.0, 0.0));
    }

    ShapeMatchEvalConfig single;
    single.topK = 10;
    single.searchRadiusPx = 20.0;
    single.minValidPoints = 1;
    single.enablePointEvaluationsForTopKOnly = false;
    single.finalRankerNumThreads = 1;
    CandidateRanker singleRanker(single);
    FinalRankerProfile singleProfile;
    const std::vector<ScoredCandidate> singleOut = singleRanker.rank(candidates, model, edgeData, nullptr, &singleProfile);

    ShapeMatchEvalConfig parallel = single;
    parallel.finalRankerNumThreads = 4;
    parallel.enableParallelFinalRanking = true;
    CandidateRanker parallelRanker(parallel);
    FinalRankerProfile parallelProfile;
    const std::vector<ScoredCandidate> parallelOut = parallelRanker.rank(candidates, model, edgeData, nullptr, &parallelProfile);

    bool ok = true;
    ok &= expect(!singleOut.empty() && !parallelOut.empty(), "rank outputs should not be empty");
    ok &= expect(std::abs(singleOut.front().pose.x - parallelOut.front().pose.x) < 1e-6, "parallel top1 pose should match single-thread top1");
    ok &= expect(parallelProfile.linearScanFallbackCount == 0, "parallel rank should not linear scan");
    return ok;
}

bool testSynthetic20MpPerformance()
{
    const ShapeTemplateModel model = makeLineTemplate(80);
    const MatchPose gt = MatchPose::fromDeg(2500.0, 2000.0, 0.0);
    EdgeImageData edgeData = makeVerticalLineEdgeData(cv::Size(5000, 4000), 2500, model, gt);

    std::vector<MatchPose> candidates;
    for (int i = 0; i < 160; ++i) {
        candidates.push_back(MatchPose::fromDeg(2500.0 + static_cast<double>((i % 40) - 20),
                                                2000.0 + static_cast<double>(i / 40),
                                                0.0));
    }

    ShapeMatchEvalConfig config;
    config.topK = 10;
    config.searchRadiusPx = 30.0;
    config.minValidPoints = 1;
    config.enablePointEvaluationsForTopKOnly = false;
    config.finalRankerNumThreads = 4;
    CandidateRanker ranker(config);
    FinalRankerProfile profile;
    const auto t0 = std::chrono::steady_clock::now();
    const std::vector<ScoredCandidate> out = ranker.rank(candidates, model, edgeData, nullptr, &profile);
    const double totalMs = elapsedMsSince(t0);

    std::cout << "[FinalRankerOptimization] synthetic_20mp total_ms=" << totalMs
              << " before=" << profile.candidateCountBeforeNms
              << " after=" << profile.candidateCountAfterNms
              << " mode=" << profile.evaluationMode
              << " linear=" << profile.linearScanFallbackCount
              << " point_eval_ms=" << profile.pointEvalTimeMs << '\n';

    bool ok = true;
    ok &= expect(!out.empty(), "20MP synthetic rank output should not be empty");
    ok &= expect(profile.linearScanFallbackCount == 0, "20MP synthetic rank should not linear scan");
    ok &= expect(profile.candidateCountAfterNms <= config.targetFinalRankerInputCandidates, "20MP synthetic NMS should cap candidates");
    return ok;
}

} // namespace
} // namespace ShapeMatch

int main()
{
    bool ok = true;
    ok &= ShapeMatch::testFastNearestEdge();
    ok &= ShapeMatch::testEvaluatorTrend();
    ok &= ShapeMatch::testFinalCandidateNms();
    ok &= ShapeMatch::testParallelRankingConsistency();
    ok &= ShapeMatch::testSynthetic20MpPerformance();
    if (ok) {
        std::cout << "[FinalRankerOptimization] self-test PASSED\n";
    }
    return ok ? 0 : 1;
}
