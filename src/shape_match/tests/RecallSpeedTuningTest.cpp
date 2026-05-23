#include "shape_match/tests/RecallSpeedTuningTest.h"

#include "shape_match/coarse/CoarseShapeMatcher.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>

namespace ShapeMatch {

namespace {

struct TestCase
{
    std::string name;
    ShapeTemplateModel model;
    EdgeImageData edgeData;
    std::vector<GroundTruthInstance> gt;
};

struct RunMetrics
{
    std::string caseName;
    std::string preset;
    std::string disabledFeature;
    double totalTimeMs = 0.0;
    double fastScoreTimeMs = 0.0;
    double recall = 0.0;
    double precision = 0.0;
    double f1 = 0.0;
    bool top1Hit = false;
    bool top3Hit = false;
    bool top5Hit = false;
    bool top10Hit = false;
    int tp = 0;
    int fp = 0;
    int fn = 0;
    double avgDxy = 0.0;
    double avgDtheta = 0.0;
    int level0RawCandidates = 0;
    int level0AfterBudgetCandidates = 0;
    int level0ScoredCandidates = 0;
    int prunedCount = 0;
    double avgEvaluatedPointsPerCandidate = 0.0;
};

bool expect(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "[RecallSpeedTuning][FAIL] " << message << '\n';
    }
    return condition;
}

CoarseMatchConfig configForPreset(CoarseMatchConfig::CoarsePreset preset)
{
    CoarseMatchConfig config;
    config.applyPreset(preset);
    config.pyramidLevels = 4;
    config.topKPerLevel = 60;
    config.finalTopK = 50;
    config.beamWidth = 50;
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
    config.maxVotingCandidatesToScore = 400;
    config.nmsTranslationThresholdPx = 8.0;
    config.nmsAngleThresholdDeg = 3.0;
    config.topKPerGridCell = preset == CoarseMatchConfig::CoarsePreset::Fast ? 5 : 6;
    config.globalTopKAfterDiversity = 80;
    return config;
}

ShapeMatchEvalConfig evalConfig()
{
    ShapeMatchEvalConfig config;
    config.topK = 50;
    config.searchRadiusPx = 5.0;
    config.minValidPoints = 20;
    return config;
}

ShapeTemplateModel createLShape()
{
    ShapeTemplateModel model;
    model.templateId = "recall_l_shape";
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

EdgeImageData emptyEdgeData(cv::Size size)
{
    EdgeImageData data;
    data.imageSize = size;
    data.edgeMap = cv::Mat::zeros(size, CV_8U);
    data.gradX = cv::Mat::zeros(size, CV_32F);
    data.gradY = cv::Mat::zeros(size, CV_32F);
    data.gradMag = cv::Mat::zeros(size, CV_32F);
    return data;
}

void addInstance(EdgeImageData& dst, const ShapeTemplateModel& model, const MatchPose& pose, double normalScale = 1.0)
{
    EdgeImageData instance = EdgeImageData::createFromTemplateAndPose(model, pose, dst.imageSize);
    for (size_t i = 0; i < instance.edgePoints.size(); ++i) {
        const cv::Point2d p = instance.edgePoints[i];
        const cv::Point2d baseNormal = i < instance.edgeNormals.size() ? instance.edgeNormals[i] : cv::Point2d(1.0, 0.0);
        const cv::Point2d n = normalized(baseNormal * normalScale);
        const int x = std::clamp(static_cast<int>(std::round(p.x)), 0, dst.imageSize.width - 1);
        const int y = std::clamp(static_cast<int>(std::round(p.y)), 0, dst.imageSize.height - 1);
        dst.edgePoints.emplace_back(static_cast<double>(x), static_cast<double>(y));
        dst.edgeNormals.push_back(n);
        dst.edgeMap.at<uchar>(y, x) = 255;
        dst.gradX.at<float>(y, x) = static_cast<float>(n.x);
        dst.gradY.at<float>(y, x) = static_cast<float>(n.y);
        dst.gradMag.at<float>(y, x) = 255.0f;
    }
}

void addNoise(EdgeImageData& data, int count)
{
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> xDist(0, data.imageSize.width - 1);
    std::uniform_int_distribution<int> yDist(0, data.imageSize.height - 1);
    std::uniform_real_distribution<double> aDist(-kPi, kPi);
    for (int i = 0; i < count; ++i) {
        const int x = xDist(rng);
        const int y = yDist(rng);
        const cv::Point2d n(std::cos(aDist(rng)), std::sin(aDist(rng)));
        data.edgePoints.emplace_back(static_cast<double>(x), static_cast<double>(y));
        data.edgeNormals.push_back(n);
        data.edgeMap.at<uchar>(y, x) = 255;
        data.gradX.at<float>(y, x) = static_cast<float>(n.x);
        data.gradY.at<float>(y, x) = static_cast<float>(n.y);
        data.gradMag.at<float>(y, x) = 80.0f;
    }
}

void addLine(EdgeImageData& data, const cv::Point2d& a, const cv::Point2d& b, int samples)
{
    const cv::Point2d dir = b - a;
    const cv::Point2d normal = normalized(cv::Point2d(-dir.y, dir.x));
    for (int i = 0; i < samples; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(std::max(1, samples - 1));
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

void occlude(EdgeImageData& data, const cv::Rect& rect)
{
    std::vector<cv::Point2d> points;
    std::vector<cv::Point2d> normals;
    for (size_t i = 0; i < data.edgePoints.size(); ++i) {
        const cv::Point2d p = data.edgePoints[i];
        const int x = std::clamp(static_cast<int>(std::round(p.x)), 0, data.imageSize.width - 1);
        const int y = std::clamp(static_cast<int>(std::round(p.y)), 0, data.imageSize.height - 1);
        if (rect.contains(cv::Point(x, y))) {
            data.edgeMap.at<uchar>(y, x) = 0;
            data.gradX.at<float>(y, x) = 0.0f;
            data.gradY.at<float>(y, x) = 0.0f;
            data.gradMag.at<float>(y, x) = 0.0f;
        } else {
            points.push_back(p);
            normals.push_back(i < data.edgeNormals.size() ? data.edgeNormals[i] : cv::Point2d(1.0, 0.0));
        }
    }
    data.edgePoints = std::move(points);
    data.edgeNormals = std::move(normals);
}

std::vector<TestCase> buildCases()
{
    const ShapeTemplateModel rect = ShapeTemplateModel::createSyntheticRectangle("recall_rect", 100.0, 70.0, 24);
    const ShapeTemplateModel lShape = createLShape();
    std::vector<TestCase> cases;

    {
        TestCase tc;
        tc.name = "four_target";
        tc.model = lShape;
        tc.edgeData = emptyEdgeData(cv::Size(860, 640));
        tc.gt = {{"obj_001", MatchPose::fromDeg(185.0, 150.0, -12.0)},
                 {"obj_002", MatchPose::fromDeg(675.0, 150.0, 26.0)},
                 {"obj_003", MatchPose::fromDeg(185.0, 490.0, 64.0)},
                 {"obj_004", MatchPose::fromDeg(675.0, 490.0, -38.0)}};
        for (const auto& gt : tc.gt) addInstance(tc.edgeData, tc.model, gt.pose);
        cases.push_back(std::move(tc));
    }
    {
        TestCase tc;
        tc.name = "reversed_polarity";
        tc.model = rect;
        tc.edgeData = emptyEdgeData(cv::Size(720, 540));
        tc.gt = {{"obj_001", MatchPose::fromDeg(340.0, 255.0, 18.0)}};
        addInstance(tc.edgeData, tc.model, tc.gt.front().pose, -1.0);
        cases.push_back(std::move(tc));
    }
    {
        TestCase tc;
        tc.name = "line_interference";
        tc.model = lShape;
        tc.gt = {{"obj_001", MatchPose::fromDeg(365.0, 280.0, -22.0)}};
        tc.edgeData = EdgeImageData::createFromTemplateAndPose(tc.model, tc.gt.front().pose, cv::Size(760, 560));
        addLine(tc.edgeData, cv::Point2d(40.0, 500.0), cv::Point2d(720.0, 70.0), 900);
        cases.push_back(std::move(tc));
    }
    {
        TestCase tc;
        tc.name = "occlusion";
        tc.model = rect;
        tc.gt = {{"obj_001", MatchPose::fromDeg(350.0, 270.0, 24.0)}};
        tc.edgeData = EdgeImageData::createFromTemplateAndPose(tc.model, tc.gt.front().pose, cv::Size(720, 540));
        occlude(tc.edgeData, cv::Rect(330, 240, 140, 100));
        cases.push_back(std::move(tc));
    }
    {
        TestCase tc;
        tc.name = "noise";
        tc.model = rect;
        tc.gt = {{"obj_001", MatchPose::fromDeg(300.0, 260.0, -31.0)}};
        tc.edgeData = EdgeImageData::createFromTemplateAndPose(tc.model, tc.gt.front().pose, cv::Size(720, 540));
        addNoise(tc.edgeData, 500);
        cases.push_back(std::move(tc));
    }
    return cases;
}

RunMetrics collectMetrics(const std::string& caseName,
                          const std::string& preset,
                          const std::string& disabledFeature,
                          const CoarseMatchReport& report)
{
    RunMetrics m;
    m.caseName = caseName;
    m.preset = preset;
    m.disabledFeature = disabledFeature;
    m.totalTimeMs = report.totalTimeMs;
    m.fastScoreTimeMs = report.profile.fastScoreTimeMs;
    m.recall = report.multiTargetEval.recall;
    m.precision = report.multiTargetEval.precision;
    m.f1 = report.multiTargetEval.f1;
    m.top1Hit = report.multiTargetEval.top1Hit;
    m.top3Hit = report.multiTargetEval.top3Hit;
    m.top5Hit = report.multiTargetEval.top5Hit;
    m.top10Hit = report.multiTargetEval.top10Hit;
    m.tp = report.multiTargetEval.truePositive;
    m.fp = report.multiTargetEval.falsePositive;
    m.fn = report.multiTargetEval.falseNegative;
    m.prunedCount = report.profile.upperBoundPrunedCount;
    m.avgEvaluatedPointsPerCandidate = report.profile.avgEvaluatedPointsPerCandidate;
    m.level0RawCandidates = report.profile.level0CandidateCountBefore;
    m.level0AfterBudgetCandidates = report.profile.level0CandidateCountAfterBudget;
    m.level0ScoredCandidates = report.profile.level0ScoredCandidateCount;
    double dxySum = 0.0;
    double dthetaSum = 0.0;
    int count = 0;
    for (const ScoredCandidate& c : report.finalRankedCandidates) {
        if (c.poseError.hasGroundTruth && c.poseError.poseOk) {
            dxySum += c.poseError.dxy;
            dthetaSum += c.poseError.dthetaDeg;
            ++count;
        }
    }
    m.avgDxy = count > 0 ? dxySum / count : 0.0;
    m.avgDtheta = count > 0 ? dthetaSum / count : 0.0;
    return m;
}

void writeMetricsCsvRow(std::ofstream& file, const RunMetrics& m, bool includePreset)
{
    file << m.caseName << ',';
    if (includePreset) {
        file << m.preset << ',';
    } else {
        file << m.disabledFeature << ',';
    }
    file << m.totalTimeMs << ',' << m.fastScoreTimeMs << ',' << m.recall << ',';
    if (includePreset) {
        file << m.precision << ',' << m.f1 << ',';
    }
    file << (m.top1Hit ? "true" : "false") << ',' << (m.top3Hit ? "true" : "false") << ','
         << (m.top5Hit ? "true" : "false") << ',' << (m.top10Hit ? "true" : "false") << ','
         << m.tp << ',' << m.fp << ',' << m.fn << ',';
    if (includePreset) {
        file << m.avgDxy << ',' << m.avgDtheta << ','
             << m.level0RawCandidates << ',' << m.level0AfterBudgetCandidates << ',' << m.level0ScoredCandidates << ',';
    } else {
        file << m.level0ScoredCandidates << ',';
    }
    file << m.prunedCount << ',' << m.avgEvaluatedPointsPerCandidate << '\n';
}

nlohmann::json metricsJson(const RunMetrics& m)
{
    return {{"case_name", m.caseName},
            {"preset", m.preset},
            {"disabled_feature", m.disabledFeature},
            {"totalTimeMs", m.totalTimeMs},
            {"fastScoreTimeMs", m.fastScoreTimeMs},
            {"recall", m.recall},
            {"precision", m.precision},
            {"f1", m.f1},
            {"top1Hit", m.top1Hit},
            {"top3Hit", m.top3Hit},
            {"top5Hit", m.top5Hit},
            {"top10Hit", m.top10Hit},
            {"TP", m.tp},
            {"FP", m.fp},
            {"FN", m.fn},
            {"avgDxy", m.avgDxy},
            {"avgDtheta", m.avgDtheta},
            {"level0RawCandidates", m.level0RawCandidates},
            {"level0AfterBudgetCandidates", m.level0AfterBudgetCandidates},
            {"level0ScoredCandidates", m.level0ScoredCandidates},
            {"prunedCount", m.prunedCount},
            {"avgEvaluatedPointsPerCandidate", m.avgEvaluatedPointsPerCandidate}};
}

CoarseMatchReport runMatcher(const TestCase& tc, const CoarseMatchConfig& config)
{
    CoarseShapeMatcher matcher(config, evalConfig());
    return matcher.match(tc.model, tc.edgeData, &tc.gt);
}

} // namespace

bool runRecallSpeedTuningTest()
{
    const auto reportsDir = std::filesystem::path("data") / "shape_match" / "reports";
    std::filesystem::create_directories(reportsDir);
    const std::vector<TestCase> cases = buildCases();
    bool ok = true;

    std::vector<RunMetrics> tuningRows;
    for (CoarseMatchConfig::CoarsePreset preset : {CoarseMatchConfig::CoarsePreset::Fast,
                                                   CoarseMatchConfig::CoarsePreset::Balanced,
                                                   CoarseMatchConfig::CoarsePreset::Recall}) {
        CoarseMatchConfig config = configForPreset(preset);
        const std::string presetName = CoarseMatchConfig::presetName(preset);
        for (const TestCase& tc : cases) {
            CoarseMatchReport report = runMatcher(tc, config);
            tuningRows.push_back(collectMetrics(tc.name, presetName, "", report));
        }
    }

    std::ofstream tuningCsv(reportsDir / "latest_recall_speed_tuning.csv");
    tuningCsv << "case_name,preset,totalTimeMs,fastScoreTimeMs,recall,precision,f1,top1Hit,top3Hit,top5Hit,top10Hit,TP,FP,FN,avgDxy,avgDtheta,level0RawCandidates,level0AfterBudgetCandidates,level0ScoredCandidates,upperBoundPrunedCount,avgEvaluatedPointsPerCandidate\n";
    nlohmann::json tuningJson;
    for (const RunMetrics& row : tuningRows) {
        writeMetricsCsvRow(tuningCsv, row, true);
        tuningJson["rows"].push_back(metricsJson(row));
    }
    std::ofstream(reportsDir / "latest_recall_speed_tuning.json") << tuningJson.dump(2);

    struct FeatureToggle {
        std::string name;
        void (*disable)(CoarseMatchConfig&);
    };
    const std::vector<FeatureToggle> toggles{
        {"enableCandidateBudgetPolicy", [](CoarseMatchConfig& c) { c.enableCandidateBudgetPolicy = false; c.enableAdaptiveCandidateBudget = false; }},
        {"enableAdaptiveRefineWindow", [](CoarseMatchConfig& c) { c.enableAdaptiveRefineWindow = false; }},
        {"enableParentDiversityBeforeRefine", [](CoarseMatchConfig& c) { c.enableParentDiversityBeforeRefine = false; }},
        {"enableRotatedTemplateCache", [](CoarseMatchConfig& c) { c.enableRotatedTemplateCache = false; }},
        {"enableParallelCandidateScoring", [](CoarseMatchConfig& c) { c.enableParallelCandidateScoring = false; c.numScoringThreads = 1; }},
        {"enableGreedyUpperBoundPruning", [](CoarseMatchConfig& c) { c.enableGreedyUpperBoundPruning = false; }},
        {"enableSpatialDiversity", [](CoarseMatchConfig& c) { c.enableSpatialDiversity = false; }},
        {"enableDistanceFieldScoring", [](CoarseMatchConfig& c) { c.enableDistanceFieldScoring = false; }}
    };

    std::ofstream diagCsv(reportsDir / "latest_recall_regression_diagnostic.csv");
    diagCsv << "case_name,disabled_feature,totalTimeMs,fastScoreTimeMs,recall,top1Hit,top3Hit,top5Hit,top10Hit,TP,FP,FN,level0Candidates,prunedCount,avgEvaluatedPointsPerCandidate\n";
    nlohmann::json diagJson;
    const std::vector<TestCase> diagCases{cases[0], cases[1], cases[2], cases[4]};
    for (const TestCase& tc : diagCases) {
        CoarseMatchConfig base = configForPreset(CoarseMatchConfig::CoarsePreset::Fast);
        CoarseMatchReport baseReport = runMatcher(tc, base);
        RunMetrics baseMetrics = collectMetrics(tc.name, "Fast", "none", baseReport);
        writeMetricsCsvRow(diagCsv, baseMetrics, false);
        diagJson["rows"].push_back(metricsJson(baseMetrics));
        for (const FeatureToggle& toggle : toggles) {
            CoarseMatchConfig config = base;
            toggle.disable(config);
            CoarseMatchReport report = runMatcher(tc, config);
            RunMetrics metrics = collectMetrics(tc.name, "Fast", toggle.name, report);
            writeMetricsCsvRow(diagCsv, metrics, false);
            diagJson["rows"].push_back(metricsJson(metrics));
        }
    }
    std::ofstream(reportsDir / "latest_recall_regression_diagnostic.json") << diagJson.dump(2);

    auto averageRecall = [&](const std::string& preset) {
        double sum = 0.0;
        int count = 0;
        for (const RunMetrics& row : tuningRows) {
            if (row.preset == preset) {
                sum += row.recall;
                ++count;
            }
        }
        return count > 0 ? sum / count : 0.0;
    };
    auto averageTime = [&](const std::string& preset) {
        double sum = 0.0;
        int count = 0;
        for (const RunMetrics& row : tuningRows) {
            if (row.preset == preset) {
                sum += row.totalTimeMs;
                ++count;
            }
        }
        return count > 0 ? sum / count : 0.0;
    };

    const double fastRecall = averageRecall("Fast");
    const double balancedRecall = averageRecall("Balanced");
    const double recallRecall = averageRecall("Recall");
    const double fastTime = averageTime("Fast");
    const double balancedTime = averageTime("Balanced");
    const double recallTime = averageTime("Recall");
    ok &= expect(balancedRecall + 1e-9 >= fastRecall, "Balanced recall should not be lower than Fast on average");
    ok &= expect(recallRecall + 1e-9 >= balancedRecall, "Recall preset should not be lower than Balanced on average");
    ok &= expect(fastTime <= recallTime * 1.25, "Fast should remain faster than Recall within tolerance");

    std::cout << "[RecallSpeedTuning] avg Fast recall=" << fastRecall << " timeMs=" << fastTime << '\n';
    std::cout << "[RecallSpeedTuning] avg Balanced recall=" << balancedRecall << " timeMs=" << balancedTime << '\n';
    std::cout << "[RecallSpeedTuning] avg Recall recall=" << recallRecall << " timeMs=" << recallTime << '\n';
    std::cout << "[RecallSpeedTuning] reports_dir=" << reportsDir.string() << '\n';
    if (ok) {
        std::cout << "[RecallSpeedTuning] self-test PASSED\n";
    }
    return ok;
}

} // namespace ShapeMatch
