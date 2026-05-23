#include "shape_match/tests/FastPoseScorerSelfTest.h"

#include "shape_match/coarse/DistanceFieldBuilder.h"
#include "shape_match/coarse/FastPoseScorer.h"
#include "shape_match/coarse/ParallelCandidateScorer.h"
#include "shape_match/coarse/RotatedTemplateCache.h"
#include "shape_match/coarse/TemplatePointSoA.h"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

namespace ShapeMatch {

namespace {

bool expect(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "[FastPoseScorer][FAIL] " << message << '\n';
    }
    return condition;
}

EdgeImageData createVerticalLineEdgeData()
{
    EdgeImageData data;
    data.imageSize = cv::Size(101, 81);
    data.edgeMap = cv::Mat::zeros(data.imageSize, CV_8U);
    data.gradX = cv::Mat::zeros(data.imageSize, CV_32F);
    data.gradY = cv::Mat::zeros(data.imageSize, CV_32F);
    data.gradMag = cv::Mat::zeros(data.imageSize, CV_32F);
    for (int y = 5; y < data.imageSize.height - 5; ++y) {
        const int x = 50;
        data.edgeMap.at<uchar>(y, x) = 255;
        data.gradX.at<float>(y, x) = 1.0f;
        data.gradY.at<float>(y, x) = 0.0f;
        data.gradMag.at<float>(y, x) = 255.0f;
        data.edgePoints.emplace_back(static_cast<double>(x), static_cast<double>(y));
        data.edgeNormals.emplace_back(1.0, 0.0);
    }
    return data;
}

CoarseMatchConfig scorerConfig(bool distanceField)
{
    CoarseMatchConfig config;
    config.enableDistanceFieldScoring = distanceField;
    config.enableLocalSearchFallback = true;
    config.enableGreedyUpperBoundPruning = true;
    config.enablePointOrdering = true;
    config.minTemplatePointsForScore = 10;
    config.maxDistanceForScore = 10.0f;
    config.distanceScoreSigma = 2.0f;
    config.orientationScoreSigmaDeg = 25.0f;
    config.fastDistanceSigma = 2.0;
    config.fastAngleSigmaDeg = 25.0;
    config.upperBoundCheckInterval = 8;
    config.minEvaluatedPointsBeforePruning = 16;
    config.upperBoundMargin = 0.02f;
    return config;
}

double elapsedUsSince(std::chrono::steady_clock::time_point t0)
{
    return std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
}

struct CompareRow
{
    int id = 0;
    MatchPose pose;
    CoarseCandidate local;
    CoarseCandidate df;
    double localUs = 0.0;
    double dfUs = 0.0;
};

} // namespace

bool runFastPoseScorerSelfTest()
{
    bool ok = true;
    const std::filesystem::path reportsDir = std::filesystem::path("data") / "shape_match" / "reports";
    std::filesystem::create_directories(reportsDir);

    {
        EdgeImageData line = createVerticalLineEdgeData();
        DistanceFieldBuilder builder;
        ok &= expect(builder.build(line, scorerConfig(true)), "distance field should build for vertical line");
        ok &= expect(line.hasDistanceField, "hasDistanceField should be true");
        ok &= expect(std::abs(line.distanceMap.at<float>(40, 50)) <= 0.01f, "edge distance should be zero on edge");
        ok &= expect(line.distanceMap.at<float>(40, 55) > line.distanceMap.at<float>(40, 52), "distance should increase away from edge");
        ok &= expect(line.nearestEdgeX.at<int>(40, 55) == 50, "nearest edge x should point to vertical line");
    }

    ShapeTemplateModel model = ShapeTemplateModel::createSyntheticRectangle("fast_scorer_rect", 100.0, 70.0, 32);
    const MatchPose gt = MatchPose::fromDeg(300.0, 220.0, 12.0);
    EdgeImageData edgeData = EdgeImageData::createFromTemplateAndPose(model, gt, cv::Size(640, 480));
    DistanceFieldBuilder builder;
    ok &= expect(builder.build(edgeData, scorerConfig(true)), "distance field should build for synthetic rectangle");

    const std::vector<MatchPose> poses{
        gt,
        MatchPose::fromDeg(gt.x + 2.0, gt.y, gt.thetaDeg()),
        MatchPose::fromDeg(gt.x + 5.0, gt.y, gt.thetaDeg()),
        MatchPose::fromDeg(gt.x + 10.0, gt.y, gt.thetaDeg()),
        MatchPose::fromDeg(gt.x, gt.y, gt.thetaDeg() + 5.0),
        MatchPose::fromDeg(80.0, 80.0, -70.0)
    };

    FastPoseScorer localScorer(scorerConfig(false));
    FastPoseScorer dfScorer(scorerConfig(true));
    TemplatePointSoA points = TemplatePointSoABuilder().build(model, true);
    ok &= expect(points.size() == model.pointCount(), "SoA point count should match model");
    ok &= expect(points.orderedIndices.size() == model.points.size(), "SoA ordered indices should cover all points");
    ok &= expect(points.totalWeight > 0.0f, "SoA total weight should be positive");
    for (size_t i = 0; i < model.points.size(); ++i) {
        ok &= expect(std::abs(points.x[i] - static_cast<float>(model.points[i].position.x)) < 1e-5f, "SoA x should match model");
        ok &= expect(std::abs(points.y[i] - static_cast<float>(model.points[i].position.y)) < 1e-5f, "SoA y should match model");
    }
    std::vector<int> sortedIndices = points.orderedIndices;
    std::sort(sortedIndices.begin(), sortedIndices.end());
    for (int i = 0; i < static_cast<int>(sortedIndices.size()); ++i) {
        ok &= expect(sortedIndices[static_cast<size_t>(i)] == i, "SoA ordered indices should be a permutation");
    }

    RotatedTemplateCache cache;
    ok &= expect(cache.build(points, buildUniformThetaBinsRad(-180.0, 180.0, 1.0)), "rotated template cache should build");
    const RotatedTemplateView* view12 = cache.findNearest(static_cast<float>(gt.theta));
    ok &= expect(view12 != nullptr, "rotated template cache should find nearest theta");
    if (view12) {
        const float c = std::cos(static_cast<float>(gt.theta));
        const float s = std::sin(static_cast<float>(gt.theta));
        double maxRotErr = 0.0;
        for (int i = 0; i < points.size(); ++i) {
            const float rx = points.x[static_cast<size_t>(i)] * c - points.y[static_cast<size_t>(i)] * s;
            const float ry = points.x[static_cast<size_t>(i)] * s + points.y[static_cast<size_t>(i)] * c;
            maxRotErr = std::max(maxRotErr,
                                 std::hypot(static_cast<double>(view12->rx[static_cast<size_t>(i)] - rx),
                                            static_cast<double>(view12->ry[static_cast<size_t>(i)] - ry)));
        }
        ok &= expect(maxRotErr < 0.05, "rotated cache should match direct rotation for cached theta");
    }

    std::vector<CompareRow> rows;
    rows.reserve(poses.size());
    for (size_t i = 0; i < poses.size(); ++i) {
        CompareRow row;
        row.id = static_cast<int>(i);
        row.pose = poses[i];
        auto t0 = std::chrono::steady_clock::now();
        row.local = localScorer.scorePose(model, edgeData, row.pose, 0);
        row.localUs = elapsedUsSince(t0);
        t0 = std::chrono::steady_clock::now();
        row.df = dfScorer.scorePose(model, edgeData, row.pose, 0);
        row.dfUs = elapsedUsSince(t0);
        CoarseCandidate optimized = dfScorer.scorePoseFast(points, cache, edgeData, row.pose, 0, FastScoreContext{});
        ok &= expect(std::abs(optimized.fastScore - row.df.fastScore) < 0.08, "SoA/cache score should stay close to old DF score");
        rows.push_back(row);
    }

    std::ofstream compare(reportsDir / "latest_fast_scorer_compare.csv");
    compare << "candidate_id,pose_x,pose_y,theta_deg,local_score,distance_field_score,score_diff,"
               "local_time_us,distance_field_time_us,matched_points_local,matched_points_df,coverage_local,coverage_df\n";
    for (const CompareRow& row : rows) {
        compare << row.id << ','
                << row.pose.x << ',' << row.pose.y << ',' << row.pose.thetaDeg() << ','
                << row.local.fastScore << ',' << row.df.fastScore << ','
                << (row.df.fastScore - row.local.fastScore) << ','
                << row.localUs << ',' << row.dfUs << ','
                << row.local.matchedPoints << ',' << row.df.matchedPoints << ','
                << row.local.coverageApprox << ',' << row.df.coverageApprox << '\n';
    }

    ok &= expect(rows[0].df.fastScore >= rows[1].df.fastScore - 0.03, "GT DF score should be highest or close");
    ok &= expect(rows[1].df.fastScore >= rows[2].df.fastScore - 0.03, "2px DF score should be >= 5px trend");
    ok &= expect(rows[2].df.fastScore >= rows[3].df.fastScore - 0.03, "5px DF score should be >= 10px trend");
    ok &= expect(rows[5].df.fastScore < rows[0].df.fastScore, "wrong pose should score below GT");
    ok &= expect(rows[0].df.fastScore > 0.5 && rows[0].local.fastScore > 0.5, "both scorers should accept GT");

    CoarseCandidate optimizedGt = dfScorer.scorePoseFast(points, cache, edgeData, gt, 0, FastScoreContext{});
    CoarseCandidate optimizedBad = dfScorer.scorePoseFast(points, cache, edgeData, MatchPose::fromDeg(80.0, 80.0, -70.0), 0, FastScoreContext{});
    ok &= expect(optimizedGt.fastScore > optimizedBad.fastScore, "SoA/cache scorer should rank GT above bad pose");

    FastScoreContext pruningContext;
    pruningContext.hasTopKThreshold = true;
    pruningContext.currentTopKMinScore = 0.55f;
    CoarseCandidate gtCandidate = dfScorer.scorePose(model, edgeData, gt, 0, pruningContext);
    CoarseCandidate badCandidate = dfScorer.scorePose(model, edgeData, MatchPose::fromDeg(30.0, 40.0, 0.0), 0, pruningContext);
    ok &= expect(!gtCandidate.rejectedByEarlyExit, "GT candidate should not be upper-bound pruned");
    ok &= expect(badCandidate.rejectedByEarlyExit, "bad candidate should be upper-bound pruned");
    ok &= expect(badCandidate.evaluatedPoints < badCandidate.totalPointCount, "pruned candidate should evaluate fewer points");

    std::vector<MatchPose> manyPoses;
    for (int dy = -10; dy <= 10; ++dy) {
        for (int dx = -10; dx <= 10; ++dx) {
            manyPoses.push_back(MatchPose::fromDeg(gt.x + dx, gt.y + dy, gt.thetaDeg()));
        }
    }
    CoarseMatchConfig oneThread = scorerConfig(true);
    oneThread.enableParallelCandidateScoring = false;
    CoarseMatchConfig multiThread = scorerConfig(true);
    multiThread.enableParallelCandidateScoring = true;
    multiThread.numScoringThreads = 4;
    multiThread.minCandidatesForParallelScoring = 1;
    ParallelScoringResult serialResult = ParallelCandidateScorer(oneThread).scoreCandidates(manyPoses, points, cache, edgeData, 0, 20);
    ParallelScoringResult parallelResult = ParallelCandidateScorer(multiThread).scoreCandidates(manyPoses, points, cache, edgeData, 0, 20);
    auto bestPose = [](std::vector<CoarseCandidate> candidates) {
        std::sort(candidates.begin(), candidates.end(), [](const CoarseCandidate& a, const CoarseCandidate& b) {
            return a.fastScore > b.fastScore;
        });
        return candidates.empty() ? MatchPose{} : candidates.front().pose;
    };
    const MatchPose serialBest = bestPose(serialResult.candidates);
    const MatchPose parallelBest = bestPose(parallelResult.candidates);
    ok &= expect(std::hypot(serialBest.x - parallelBest.x, serialBest.y - parallelBest.y) <= 1.0,
                 "parallel scoring top pose should match serial");
    ok &= expect(parallelResult.numThreads > 1, "parallel scorer should use multiple threads when configured");

    if (ok) {
        std::cout << "[FastPoseScorer] self-test PASSED\n";
        std::cout << "[FastPoseScorer] compare_csv=" << (reportsDir / "latest_fast_scorer_compare.csv").string() << '\n';
    }
    return ok;
}

} // namespace ShapeMatch
