#include "shape_match/tests/CoarseShapeMatcherSelfTest.h"

#include "shape_match/coarse/CoarseMatchDebugOverlay.h"
#include "shape_match/coarse/CoarseShapeMatcher.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <random>
#include <set>

namespace ShapeMatch {

namespace {

bool expect(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "[CoarseShapeMatcher][FAIL] " << message << '\n';
    }
    return condition;
}

CoarseMatchConfig testConfig()
{
    CoarseMatchConfig config;
    config.pyramidLevels = 4;
    config.topKPerLevel = 20;
    config.finalTopK = 20;
    config.beamWidth = 15;
    config.coarseAngleStepDeg = 10.0;
    config.fineAngleStepDeg = 2.0;
    config.coarseTranslationStepPx = 8;
    config.fineTranslationStepPx = 1;
    config.localRefineRadiusPx = 9;
    config.localRefineAngleRadiusDeg = 7;
    config.maxTemplatePointsPerLevel = 50;
    config.minTemplatePointsForScore = 20;
    config.fastDistanceSigma = 1.5;
    config.maxCandidatesEvaluatedPerLevel = 60000;
    config.enableOrientationVoting = true;
    config.minVotingCandidates = 8;
    config.maxVotingCandidatesToScore = 300;
    config.nmsTranslationThresholdPx = 8.0;
    config.nmsAngleThresholdDeg = 3.0;
    return config;
}

ShapeMatchEvalConfig evalConfig()
{
    ShapeMatchEvalConfig config;
    config.topK = 20;
    config.searchRadiusPx = 5.0;
    config.minValidPoints = 20;
    return config;
}

ShapeTemplateModel createSyntheticLShape()
{
    ShapeTemplateModel model;
    model.templateId = "coarse_l_shape";
    model.origin = cv::Point2d(0.0, 0.0);
    int pointId = 0;
    auto addPoint = [&](const cv::Point2d& pos, const cv::Point2d& normal, const cv::Point2d& tangent, int chainId) {
        TemplatePoint p;
        p.id = pointId++;
        p.position = pos;
        p.normal = normalized(normal);
        p.tangent = normalized(tangent);
        p.gradientDir = p.normal;
        p.gradientMag = 255.0;
        p.weight = 1.0;
        p.chainId = chainId;
        p.polarity = EdgePolarity::DarkToBright;
        model.points.push_back(p);
    };

    constexpr int samples = 28;
    for (int i = 0; i < samples; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(samples - 1);
        addPoint(cv::Point2d(-55.0 + 110.0 * t, -36.0), cv::Point2d(0.0, -1.0), cv::Point2d(1.0, 0.0), 0);
        addPoint(cv::Point2d(-55.0, -36.0 + 86.0 * t), cv::Point2d(-1.0, 0.0), cv::Point2d(0.0, 1.0), 1);
        addPoint(cv::Point2d(-55.0 + 65.0 * t, 50.0), cv::Point2d(0.0, 1.0), cv::Point2d(1.0, 0.0), 2);
        addPoint(cv::Point2d(10.0, 50.0 - 42.0 * t), cv::Point2d(1.0, 0.0), cv::Point2d(0.0, -1.0), 3);
    }
    model.computeBoundingBox();
    return model;
}

void addInstanceWithNormalScale(EdgeImageData& dst, const ShapeTemplateModel& model, const MatchPose& pose, double normalScale)
{
    EdgeImageData instance = EdgeImageData::createFromTemplateAndPose(model, pose, dst.imageSize);
    for (size_t i = 0; i < instance.edgePoints.size(); ++i) {
        const cv::Point2d p = instance.edgePoints[i];
        const cv::Point2d baseNormal = i < instance.edgeNormals.size() ? instance.edgeNormals[i] : cv::Point2d(1.0, 0.0);
        const cv::Point2d n = normalized(cv::Point2d(baseNormal.x * normalScale, baseNormal.y * normalScale));
        dst.edgePoints.push_back(p);
        dst.edgeNormals.push_back(n);
        const int x = std::clamp(static_cast<int>(std::round(p.x)), 0, dst.imageSize.width - 1);
        const int y = std::clamp(static_cast<int>(std::round(p.y)), 0, dst.imageSize.height - 1);
        dst.edgeMap.at<uchar>(y, x) = 255;
        dst.gradX.at<float>(y, x) = static_cast<float>(n.x);
        dst.gradY.at<float>(y, x) = static_cast<float>(n.y);
        dst.gradMag.at<float>(y, x) = 255.0f;
    }
}

void addInstance(EdgeImageData& dst, const ShapeTemplateModel& model, const MatchPose& pose)
{
    addInstanceWithNormalScale(dst, model, pose, 1.0);
}

EdgeImageData createEmptyEdgeData(cv::Size size)
{
    EdgeImageData data;
    data.imageSize = size;
    data.edgeMap = cv::Mat::zeros(size, CV_8U);
    data.gradX = cv::Mat::zeros(size, CV_32F);
    data.gradY = cv::Mat::zeros(size, CV_32F);
    data.gradMag = cv::Mat::zeros(size, CV_32F);
    return data;
}

void removeOcclusion(EdgeImageData& data, const cv::Rect& rect)
{
    std::vector<cv::Point2d> points;
    std::vector<cv::Point2d> normals;
    for (size_t i = 0; i < data.edgePoints.size(); ++i) {
        const cv::Point2d p = data.edgePoints[i];
        if (rect.contains(cv::Point(static_cast<int>(std::round(p.x)), static_cast<int>(std::round(p.y))))) {
            const int x = std::clamp(static_cast<int>(std::round(p.x)), 0, data.imageSize.width - 1);
            const int y = std::clamp(static_cast<int>(std::round(p.y)), 0, data.imageSize.height - 1);
            data.edgeMap.at<uchar>(y, x) = 0;
            data.gradX.at<float>(y, x) = 0.0f;
            data.gradY.at<float>(y, x) = 0.0f;
            data.gradMag.at<float>(y, x) = 0.0f;
            continue;
        }
        points.push_back(p);
        normals.push_back(i < data.edgeNormals.size() ? data.edgeNormals[i] : cv::Point2d(1.0, 0.0));
    }
    data.edgePoints = std::move(points);
    data.edgeNormals = std::move(normals);
}

void addNoiseEdges(EdgeImageData& data, int count)
{
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> xDist(0, data.imageSize.width - 1);
    std::uniform_int_distribution<int> yDist(0, data.imageSize.height - 1);
    std::uniform_real_distribution<double> aDist(-kPi, kPi);
    for (int i = 0; i < count; ++i) {
        const int x = xDist(rng);
        const int y = yDist(rng);
        const double a = aDist(rng);
        const cv::Point2d n(std::cos(a), std::sin(a));
        data.edgePoints.emplace_back(static_cast<double>(x), static_cast<double>(y));
        data.edgeNormals.push_back(n);
        data.edgeMap.at<uchar>(y, x) = 255;
        data.gradX.at<float>(y, x) = static_cast<float>(n.x);
        data.gradY.at<float>(y, x) = static_cast<float>(n.y);
        data.gradMag.at<float>(y, x) = 80.0f;
    }
}

void addLineInterference(EdgeImageData& data, const cv::Point2d& a, const cv::Point2d& b, int samples)
{
    const cv::Point2d dir = b - a;
    const cv::Point2d normal = normalized(cv::Point2d(-dir.y, dir.x));
    samples = std::max(2, samples);
    for (int i = 0; i < samples; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(samples - 1);
        const cv::Point2d p = a + dir * t;
        if (!data.isInside(p)) {
            continue;
        }
        const int x = std::clamp(static_cast<int>(std::round(p.x)), 0, data.imageSize.width - 1);
        const int y = std::clamp(static_cast<int>(std::round(p.y)), 0, data.imageSize.height - 1);
        data.edgePoints.emplace_back(static_cast<double>(x), static_cast<double>(y));
        data.edgeNormals.push_back(normal);
        data.edgeMap.at<uchar>(y, x) = 255;
        data.gradX.at<float>(y, x) = static_cast<float>(normal.x);
        data.gradY.at<float>(y, x) = static_cast<float>(normal.y);
        data.gradMag.at<float>(y, x) = 255.0f;
    }
}

bool anyPoseOk(const CoarseMatchReport& report, double* bestDxy = nullptr, double* bestDtheta = nullptr)
{
    double dxy = 1e9;
    double dtheta = 1e9;
    bool ok = false;
    for (const ScoredCandidate& c : report.finalRankedCandidates) {
        if (c.poseError.hasGroundTruth) {
            dxy = std::min(dxy, c.poseError.dxy);
            dtheta = std::min(dtheta, c.poseError.dthetaDeg);
            ok = ok || c.poseError.poseOk;
        }
    }
    if (bestDxy) {
        *bestDxy = dxy;
    }
    if (bestDtheta) {
        *bestDtheta = dtheta;
    }
    return ok;
}

int occupiedQuadrants(const CoarseMatchReport& report, const cv::Size& imageSize)
{
    std::set<std::pair<int, int>> cells;
    for (const ScoredCandidate& c : report.finalRankedCandidates) {
        const int col = c.pose.x < imageSize.width * 0.5 ? 0 : 1;
        const int row = c.pose.y < imageSize.height * 0.5 ? 0 : 1;
        cells.insert({row, col});
    }
    return static_cast<int>(cells.size());
}

double nearestDxyToGt(const CoarseMatchReport& report, const MatchPose& gtPose)
{
    double best = 1e9;
    for (const ScoredCandidate& c : report.finalRankedCandidates) {
        const double dx = c.pose.x - gtPose.x;
        const double dy = c.pose.y - gtPose.y;
        best = std::min(best, std::sqrt(dx * dx + dy * dy));
    }
    return best;
}

double nearestInitialDxyToGt(const CoarseMatchReport& report, const MatchPose& gtPose)
{
    if (report.levels.empty()) {
        return 1e9;
    }
    const CoarseMatchLevelResult& initial = report.levels.front();
    double best = 1e9;
    const double scaleToBase = 1.0 / std::max(1e-12, std::pow(0.5, initial.level));
    for (const CoarseCandidate& c : initial.topCandidates) {
        const double x = c.pose.x * scaleToBase;
        const double y = c.pose.y * scaleToBase;
        const double dx = x - gtPose.x;
        const double dy = y - gtPose.y;
        best = std::min(best, std::sqrt(dx * dx + dy * dy));
    }
    return best;
}

bool initialHasVotingCandidates(const CoarseMatchReport& report)
{
    return !report.levels.empty()
        && std::any_of(report.levels.front().topCandidates.begin(),
                       report.levels.front().topCandidates.end(),
                       [](const CoarseCandidate& c) { return c.source == "voting"; });
}

void printTop(const CoarseMatchReport& report)
{
    for (size_t i = 0; i < report.finalRankedCandidates.size() && i < 5; ++i) {
        const ScoredCandidate& c = report.finalRankedCandidates[i];
        std::cout << "  rank " << c.rank
                  << " pose=(" << c.pose.x << ',' << c.pose.y << ',' << c.pose.thetaDeg() << ")"
                  << " score=" << c.score.finalScore
                  << " cov=" << c.score.coverageRatio;
        if (c.poseError.hasGroundTruth) {
            std::cout << " dxy=" << c.poseError.dxy << " dtheta=" << c.poseError.dthetaDeg;
        }
        std::cout << '\n';
    }
}

CoarseMatchReport runCase(const char* name,
                          const ShapeTemplateModel& model,
                          const EdgeImageData& edgeData,
                          const std::vector<GroundTruthInstance>& gt,
                          CoarseMatchConfig config = testConfig())
{
    std::cout << "\n[CoarseShapeMatcher] case=" << name << '\n';
    CoarseShapeMatcher matcher(config, evalConfig());
    CoarseMatchReport report = matcher.match(model, edgeData, &gt);
    for (const CoarseMatchLevelResult& level : report.levels) {
        std::cout << "  level " << level.level
                  << " source=" << level.candidateSource
                  << " evaluated=" << level.evaluatedCandidateCount
                  << " kept=" << level.keptCandidateCount
                  << " votingRaw=" << level.votingRawCandidateCount
                  << " votingScored=" << level.votingScoredCandidateCount
                  << " bestFast=" << (level.topCandidates.empty() ? 0.0 : level.topCandidates.front().fastScore)
                  << " elapsedMs=" << level.elapsedMs
                  << " limitHit=" << (level.candidateLimitHit ? "true" : "false") << '\n';
    }
    printTop(report);
    return report;
}

} // namespace

bool runCoarseShapeMatcherSelfTest()
{
    const ShapeTemplateModel model = ShapeTemplateModel::createSyntheticRectangle("coarse_rect", 100.0, 70.0, 24);
    const ShapeTemplateModel lShapeModel = createSyntheticLShape();
    bool ok = true;

    {
        const MatchPose gtPose = MatchPose::fromDeg(320.0, 240.0, 0.0);
        const EdgeImageData edgeData = EdgeImageData::createFromTemplateAndPose(model, gtPose, cv::Size(640, 480));
        const std::vector<GroundTruthInstance> gt{{"obj_001", gtPose}};
        const CoarseMatchReport report = runCase("single_no_rotation", model, edgeData, gt);
        double bestDxy = 0.0;
        double bestDtheta = 0.0;
        ok &= expect(anyPoseOk(report, &bestDxy, &bestDtheta), "single no-rotation true pose should enter top-K");
        ok &= expect(bestDxy <= 3.0, "single no-rotation should reach <= 3 px");
        ok &= expect(bestDtheta <= 2.0, "single no-rotation should reach <= 2 deg");
    }

    {
        const MatchPose gtPose = MatchPose::fromDeg(330.0, 250.0, 37.0);
        const EdgeImageData edgeData = EdgeImageData::createFromTemplateAndPose(model, gtPose, cv::Size(700, 520));
        const std::vector<GroundTruthInstance> gt{{"obj_001", gtPose}};
        const CoarseMatchReport report = runCase("single_rotation", model, edgeData, gt);
        ok &= expect(anyPoseOk(report), "single rotated true pose should enter top-K");
    }

    {
        EdgeImageData edgeData = createEmptyEdgeData(cv::Size(800, 620));
        const std::vector<GroundTruthInstance> gt{
            {"obj_001", MatchPose::fromDeg(220.0, 190.0, -18.0)},
            {"obj_002", MatchPose::fromDeg(560.0, 410.0, 33.0)},
            {"obj_003", MatchPose::fromDeg(520.0, 160.0, 82.0)}
        };
        for (const GroundTruthInstance& g : gt) {
            addInstance(edgeData, model, g.pose);
        }
        CoarseMatchConfig config = testConfig();
        config.finalTopK = 30;
        config.topKPerLevel = 35;
        config.beamWidth = 25;
        config.nmsTranslationThresholdPx = 12.0;
        const CoarseMatchReport report = runCase("multi_target", model, edgeData, gt, config);
        ok &= expect(report.multiTargetEval.recall > 0.0, "multi-target recall should be non-zero");
        ok &= expect(report.multiTargetEval.truePositive >= 2, "multi-target should recover at least two objects");
    }

    {
        const cv::Size imageSize(860, 640);
        EdgeImageData edgeData = createEmptyEdgeData(imageSize);
        const std::vector<GroundTruthInstance> gt{
            {"obj_001", MatchPose::fromDeg(185.0, 150.0, -12.0)},
            {"obj_002", MatchPose::fromDeg(675.0, 150.0, 26.0)},
            {"obj_003", MatchPose::fromDeg(185.0, 490.0, 64.0)},
            {"obj_004", MatchPose::fromDeg(675.0, 490.0, -38.0)}
        };
        for (const GroundTruthInstance& g : gt) {
            addInstance(edgeData, lShapeModel, g.pose);
        }
        CoarseMatchConfig config = testConfig();
        config.finalTopK = 50;
        config.topKPerLevel = 60;
        config.beamWidth = 50;
        config.topKPerGridCell = 6;
        config.globalTopKAfterDiversity = 80;
        const CoarseMatchReport report = runCase("four_target_spatial_diversity", lShapeModel, edgeData, gt, config);
        CoarseMatchConfig gridConfig = config;
        gridConfig.enableOrientationVoting = false;
        const CoarseMatchReport gridReport = runCase("four_target_spatial_diversity_grid_baseline", lShapeModel, edgeData, gt, gridConfig);
        ok &= expect(report.initialCandidateMode == "voting", "four-target case should use voting as initial candidate mode");
        ok &= expect(initialHasVotingCandidates(report), "four-target initial candidates should come from voting");
        ok &= expect(occupiedQuadrants(report, imageSize) >= 3, "spatial diversity should preserve candidates in multiple regions");
        ok &= expect(report.multiTargetEval.truePositive >= 3, "four-target spatial diversity should recover at least three objects");
        ok &= expect(report.multiTargetEval.truePositive >= gridReport.multiTargetEval.truePositive,
                     "voting four-target recall should be at least as good as grid baseline");
    }

    {
        const MatchPose gtPose = MatchPose::fromDeg(350.0, 270.0, 24.0);
        EdgeImageData edgeData = EdgeImageData::createFromTemplateAndPose(model, gtPose, cv::Size(720, 540));
        removeOcclusion(edgeData, cv::Rect(330, 240, 140, 100));
        const std::vector<GroundTruthInstance> gt{{"obj_001", gtPose}};
        const CoarseMatchReport report = runCase("occlusion", model, edgeData, gt);
        ok &= expect(!report.finalRankedCandidates.empty(), "occlusion should produce candidates");
        ok &= expect(report.finalRankedCandidates.front().score.coverageRatio < 0.95, "occlusion should lower coverage");
    }

    {
        const MatchPose gtPose = MatchPose::fromDeg(340.0, 255.0, 18.0);
        EdgeImageData edgeData = createEmptyEdgeData(cv::Size(720, 540));
        addInstanceWithNormalScale(edgeData, model, gtPose, -1.0);
        CoarseMatchConfig config = testConfig();
        config.finalTopK = 35;
        config.topKPerLevel = 40;
        config.beamWidth = 30;
        const std::vector<GroundTruthInstance> gt{{"obj_001", gtPose}};
        const CoarseMatchReport report = runCase("reversed_polarity_black_target", model, edgeData, gt, config);
        ok &= expect(nearestInitialDxyToGt(report, gtPose) <= 96.0,
                     "voting should generate an initial candidate near the polarity-reversed target");
        ok &= expect(nearestDxyToGt(report, gtPose) <= 9.0, "polarity reversal should not remove the true location from top-K");
        ok &= expect(!report.finalRankedCandidates.empty(), "polarity reversal should still produce final candidates");
    }

    {
        const MatchPose gtPose = MatchPose::fromDeg(300.0, 260.0, -31.0);
        EdgeImageData edgeData = EdgeImageData::createFromTemplateAndPose(model, gtPose, cv::Size(720, 540));
        addNoiseEdges(edgeData, 500);
        const std::vector<GroundTruthInstance> gt{{"obj_001", gtPose}};
        const CoarseMatchReport report = runCase("noise_edges", model, edgeData, gt);
        ok &= expect(anyPoseOk(report), "noise should not prevent true pose from entering top-K");
    }

    {
        const MatchPose gtPose = MatchPose::fromDeg(365.0, 280.0, -22.0);
        EdgeImageData edgeData = EdgeImageData::createFromTemplateAndPose(lShapeModel, gtPose, cv::Size(760, 560));
        addLineInterference(edgeData, cv::Point2d(40.0, 500.0), cv::Point2d(720.0, 70.0), 900);
        CoarseMatchConfig config = testConfig();
        config.finalTopK = 35;
        config.topKPerLevel = 45;
        config.beamWidth = 35;
        const std::vector<GroundTruthInstance> gt{{"obj_001", gtPose}};
        const CoarseMatchReport report = runCase("long_line_interference", lShapeModel, edgeData, gt, config);
        ok &= expect(nearestInitialDxyToGt(report, gtPose) <= 18.0,
                     "voting peaks should keep a true target candidate under line interference");
        ok &= expect(nearestDxyToGt(report, gtPose) <= 8.0, "long line interference should not occupy all top-K candidates");
    }

    {
        const MatchPose gtPose = MatchPose::fromDeg(410.0, 310.0, 17.0);
        const EdgeImageData edgeData = EdgeImageData::createFromTemplateAndPose(model, gtPose, cv::Size(820, 620));
        const std::vector<GroundTruthInstance> gt{{"obj_001", gtPose}};
        const CoarseMatchReport report = runCase("candidate_propagation", model, edgeData, gt);
        ok &= expect(report.levels.size() >= 2, "candidate propagation should produce multiple level reports");
        ok &= expect(!report.levels.front().topCandidates.empty(), "coarse level should have candidates");
        ok &= expect(!report.levels.back().topCandidates.empty(), "level 0 should have candidates");
    }

    {
        const MatchPose gtPose = MatchPose::fromDeg(500.0, 360.0, 12.0);
        const EdgeImageData edgeData = EdgeImageData::createFromTemplateAndPose(model, gtPose, cv::Size(1200, 900));
        const std::vector<GroundTruthInstance> gt{{"obj_001", gtPose}};
        CoarseMatchConfig config = testConfig();
        config.enableOrientationVoting = false;
        config.maxCandidatesEvaluatedPerLevel = 200;
        const CoarseMatchReport report = runCase("candidate_limit", model, edgeData, gt, config);
        bool limitHit = false;
        for (const CoarseMatchLevelResult& level : report.levels) {
            limitHit = limitHit || level.candidateLimitHit;
        }
        ok &= expect(limitHit, "candidate limit should be reported");
    }

    {
        const cv::Size imageSize(960, 720);
        EdgeImageData edgeData = createEmptyEdgeData(imageSize);
        const std::vector<GroundTruthInstance> gt{
            {"obj_001", MatchPose::fromDeg(210.0, 170.0, -20.0)},
            {"obj_002", MatchPose::fromDeg(740.0, 175.0, 24.0)},
            {"obj_003", MatchPose::fromDeg(220.0, 545.0, 71.0)},
            {"obj_004", MatchPose::fromDeg(745.0, 540.0, -42.0)}
        };
        for (const GroundTruthInstance& g : gt) {
            addInstance(edgeData, lShapeModel, g.pose);
        }
        CoarseMatchConfig votingConfig = testConfig();
        votingConfig.finalTopK = 50;
        votingConfig.topKPerLevel = 60;
        votingConfig.beamWidth = 50;
        votingConfig.maxVotingCandidatesToScore = 400;
        CoarseMatchConfig gridConfig = votingConfig;
        gridConfig.enableOrientationVoting = false;

        const CoarseMatchReport gridReport = runCase("perf_grid_top_level", lShapeModel, edgeData, gt, gridConfig);
        const CoarseMatchReport votingReport = runCase("perf_voting_top_level", lShapeModel, edgeData, gt, votingConfig);
        const double gridTopMs = gridReport.levels.empty() ? 0.0 : gridReport.levels.front().elapsedMs;
        const double votingTopMs = votingReport.profile.votingTimeMs
            + (votingReport.levels.empty() ? 0.0 : votingReport.levels.front().votingVerifyTimeMs);
        const double speedup = votingTopMs > 1e-6 ? gridTopMs / votingTopMs : 0.0;
        std::cout << "  top_level_perf gridMs=" << gridTopMs
                  << " votingMs=" << votingTopMs
                  << " totalVotingMs=" << votingReport.totalTimeMs
                  << " speedup=" << speedup << '\n';
        ok &= expect(votingReport.initialCandidateMode == "voting", "performance case should use voting by default");
        ok &= expect(votingTopMs < gridTopMs, "voting top level should be faster than grid top level");
        ok &= expect(votingReport.multiTargetEval.truePositive >= std::min(2, gridReport.multiTargetEval.truePositive),
                     "voting performance case should preserve multi-target recall");
    }

    {
        const MatchPose gtPose = MatchPose::fromDeg(300.0, 230.0, 16.0);
        const EdgeImageData edgeData = EdgeImageData::createFromTemplateAndPose(lShapeModel, gtPose, cv::Size(620, 460));
        const std::vector<GroundTruthInstance> gt{{"obj_001", gtPose}};
        CoarseMatchConfig localConfig = testConfig();
        localConfig.pyramidLevels = 3;
        localConfig.maxCandidatesEvaluatedPerLevel = 12000;
        localConfig.useGradientOrientation = false;
        localConfig.usePolarity = false;
        localConfig.enableDistanceFieldScoring = false;
        CoarseMatchConfig distanceConfig = localConfig;
        distanceConfig.enableDistanceFieldScoring = true;
        const CoarseMatchReport localReport = runCase("perf_local_search_scoring", lShapeModel, edgeData, gt, localConfig);
        const CoarseMatchReport distanceReport = runCase("perf_distance_field_scoring", lShapeModel, edgeData, gt, distanceConfig);
        const double localTime = localReport.profile.fastScoreTimeMs;
        const double distanceTime = distanceReport.profile.fastScoreTimeMs;
        const double ratio = distanceTime > 1e-6 ? localTime / distanceTime : 0.0;
        std::cout << "  scoring_perf localMs=" << localTime
                  << " distanceMs=" << distanceTime
                  << " speedup=" << ratio << '\n';
        ok &= expect(!distanceReport.finalRankedCandidates.empty(), "distance-field scoring should produce candidates");
        ok &= expect(distanceTime <= localTime * 1.25, "distance-field scoring should not be slower than local search on the perf case");
    }

    const CoarseMatchReport latest = runCase("overlay_smoke",
                                             model,
                                             EdgeImageData::createFromTemplateAndPose(model, MatchPose::fromDeg(320.0, 240.0, 0.0), cv::Size(640, 480)),
                                             {{"obj_001", MatchPose::fromDeg(320.0, 240.0, 0.0)}});
    CoarseMatchDebugOverlay overlayBuilder;
    const ShapeMatchOverlayData overlay = overlayBuilder.buildOverlayData(latest, model);
    ok &= expect(!overlay.empty(), "coarse overlay should be generated");
    std::cout << "\n[CoarseShapeMatcher] overlay polylines=" << overlay.polylines.size()
              << " points=" << overlay.points.size()
              << " lines=" << overlay.lines.size()
              << " texts=" << overlay.texts.size() << '\n';
    std::cout << "[CoarseShapeMatcher] reports_dir="
              << (std::filesystem::path("data") / "shape_match" / "reports").string() << '\n';

    if (ok) {
        std::cout << "[CoarseShapeMatcher] self-test PASSED\n";
    }
    return ok;
}

} // namespace ShapeMatch
