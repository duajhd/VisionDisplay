#include "shape_match/coarse/CoarseMatchConfig.h"
#include "shape_match/coarse/ImagePyramid.h"
#include "shape_match/coarse/OrientationResponseConfig.h"
#include "shape_match/coarse/ResponseCandidateGenerator.h"
#include "shape_match/coarse/TemplatePyramid.h"
#include "shape_match/core/EdgeImageData.h"
#include "shape_match/core/ShapeTemplateModel.h"
#include "shape_match/pipeline_v2/ShapeMatchPipelineV2.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <set>
#include <string>

namespace ShapeMatch {

namespace {

bool expect(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "[ResponseCandidate][FAIL] " << message << '\n';
    }
    return condition;
}

OrientationResponseConfig testConfig()
{
    OrientationResponseConfig config;
    config.orientationBinCount = 16;
    config.thetaBinCount = 36;
    config.maxTemplateResponsePoints = 80;
    config.spatialSpreadRadiusPx = 3;
    config.maxPeaksPerTheta = 80;
    config.maxTotalPeaks = 300;
    config.minResponseScore = 0.10;
    config.tileRows = 6;
    config.tileCols = 6;
    config.maxPeaksPerTile = 4;
    config.nmsRadiusPx = 6;
    config.nmsThetaRadiusBins = 1;
    return config;
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

void addInstance(EdgeImageData& dst, const ShapeTemplateModel& model, const MatchPose& pose, double mag = 255.0)
{
    EdgeImageData instance = EdgeImageData::createFromTemplateAndPose(model, pose, dst.imageSize);
    for (size_t i = 0; i < instance.edgePoints.size(); ++i) {
        const cv::Point2d p = instance.edgePoints[i];
        const cv::Point2d n = i < instance.edgeNormals.size() ? instance.edgeNormals[i] : cv::Point2d(1.0, 0.0);
        const int x = std::clamp(static_cast<int>(std::round(p.x)), 0, dst.imageSize.width - 1);
        const int y = std::clamp(static_cast<int>(std::round(p.y)), 0, dst.imageSize.height - 1);
        dst.edgePoints.emplace_back(x, y);
        dst.edgeNormals.push_back(n);
        dst.edgeMap.at<uchar>(y, x) = 255;
        dst.gradX.at<float>(y, x) = static_cast<float>(n.x);
        dst.gradY.at<float>(y, x) = static_cast<float>(n.y);
        dst.gradMag.at<float>(y, x) = static_cast<float>(mag);
    }
}

void addLineInterference(EdgeImageData& data, const cv::Point2d& a, const cv::Point2d& b, int samples)
{
    const cv::Point2d dir = b - a;
    const cv::Point2d normal = normalized(cv::Point2d(-dir.y, dir.x));
    for (int i = 0; i < samples; ++i) {
        const double t = samples <= 1 ? 0.0 : static_cast<double>(i) / static_cast<double>(samples - 1);
        const cv::Point2d p = a + dir * t;
        if (!data.isInside(p)) {
            continue;
        }
        const int x = std::clamp(static_cast<int>(std::round(p.x)), 0, data.imageSize.width - 1);
        const int y = std::clamp(static_cast<int>(std::round(p.y)), 0, data.imageSize.height - 1);
        data.edgePoints.emplace_back(x, y);
        data.edgeNormals.push_back(normal);
        data.edgeMap.at<uchar>(y, x) = 255;
        data.gradX.at<float>(y, x) = static_cast<float>(normal.x);
        data.gradY.at<float>(y, x) = static_cast<float>(normal.y);
        data.gradMag.at<float>(y, x) = 255.0f;
    }
}

double dxy(const MatchPose& a, const MatchPose& b)
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

double dthetaDeg(const MatchPose& a, const MatchPose& b)
{
    return std::abs(radToDeg(wrapToPi(a.theta - b.theta)));
}

bool hasPeakNear(const std::vector<ResponsePeak>& peaks, const MatchPose& gt, double maxDxy, double maxDtheta)
{
    for (const ResponsePeak& peak : peaks) {
        if (dxy(peak.pose, gt) <= maxDxy && dthetaDeg(peak.pose, gt) <= maxDtheta) {
            return true;
        }
    }
    return false;
}

bool hasSpatialPeakNear(const std::vector<ResponsePeak>& peaks, const MatchPose& gt, double maxDxy)
{
    for (const ResponsePeak& peak : peaks) {
        if (dxy(peak.pose, gt) <= maxDxy) {
            return true;
        }
    }
    return false;
}

int occupiedQuadrants(const std::vector<ResponsePeak>& peaks, cv::Size imageSize)
{
    std::set<std::pair<int, int>> cells;
    for (const ResponsePeak& peak : peaks) {
        const int col = peak.pose.x < imageSize.width * 0.5 ? 0 : 1;
        const int row = peak.pose.y < imageSize.height * 0.5 ? 0 : 1;
        cells.insert({row, col});
    }
    return static_cast<int>(cells.size());
}

void printReport(const char* name, const ResponseDebugReport& report)
{
    std::cout << "[ResponseCandidate] case=" << name
              << " peaks=" << report.totalPeaksAfterNms
              << " templatePoints=" << report.responseTemplatePointCount
              << " totalMs=" << report.totalMs
              << " evalMs=" << report.evaluateResponseMs << '\n';
}

} // namespace

int runResponseCandidateGeneratorSelfTest()
{
    bool ok = true;
    const ShapeTemplateModel model = ShapeTemplateModel::createSyntheticRectangle("response_rect", 48.0, 32.0, 18);
    const OrientationResponseConfig config = testConfig();
    const double thetaTolerance = 360.0 / static_cast<double>(config.thetaBinCount) + 1.0;

    {
        const MatchPose gt = MatchPose::fromDeg(105.0, 85.0, 0.0);
        const EdgeImageData edgeData = EdgeImageData::createFromTemplateAndPose(model, gt, cv::Size(220, 170));
        ResponseDebugReport report;
        ResponseCandidateGenerator generator(config);
        const std::vector<ResponsePeak> peaks = generator.generatePeaks(model, edgeData, 0, &report);
        printReport("single_no_rotation", report);
        ok &= expect(!peaks.empty(), "single no-rotation should produce peaks");
        ok &= expect(hasPeakNear(peaks, gt, 8.0, thetaTolerance), "single no-rotation should have a GT-near peak");
    }

    {
        const MatchPose gt = MatchPose::fromDeg(110.0, 90.0, 37.0);
        const EdgeImageData edgeData = EdgeImageData::createFromTemplateAndPose(model, gt, cv::Size(230, 180));
        ResponseDebugReport report;
        ResponseCandidateGenerator generator(config);
        const std::vector<ResponsePeak> peaks = generator.generatePeaks(model, edgeData, 0, &report);
        printReport("single_rotation", report);
        ok &= expect(hasPeakNear(peaks, gt, 8.0, thetaTolerance), "single rotation should have a GT-near peak");
    }

    {
        const cv::Size imageSize(360, 260);
        EdgeImageData edgeData = createEmptyEdgeData(imageSize);
        const std::vector<MatchPose> gt{
            MatchPose::fromDeg(85.0, 70.0, -10.0),
            MatchPose::fromDeg(275.0, 70.0, 28.0),
            MatchPose::fromDeg(85.0, 195.0, 62.0),
            MatchPose::fromDeg(275.0, 195.0, -38.0)
        };
        for (const MatchPose& pose : gt) {
            addInstance(edgeData, model, pose);
        }
        ResponseDebugReport report;
        ResponseCandidateGenerator generator(config);
        const std::vector<ResponsePeak> peaks = generator.generatePeaks(model, edgeData, 0, &report);
        printReport("four_targets", report);
        int hitCount = 0;
        for (const MatchPose& pose : gt) {
            if (hasSpatialPeakNear(peaks, pose, 8.0)) {
                ++hitCount;
            }
        }
        ok &= expect(hitCount >= 3, "four-target response should cover at least three GT objects");
        ok &= expect(occupiedQuadrants(peaks, imageSize) >= 3, "response peaks should cover multiple regions");
    }

    {
        const MatchPose gt = MatchPose::fromDeg(125.0, 95.0, -22.0);
        EdgeImageData edgeData = EdgeImageData::createFromTemplateAndPose(model, gt, cv::Size(260, 190));
        addLineInterference(edgeData, cv::Point2d(20.0, 170.0), cv::Point2d(240.0, 20.0), 500);
        ResponseDebugReport report;
        ResponseCandidateGenerator generator(config);
        const std::vector<ResponsePeak> peaks = generator.generatePeaks(model, edgeData, 0, &report);
        printReport("strong_line_interference", report);
        ok &= expect(hasSpatialPeakNear(peaks, gt, 10.0), "strong line interference should not remove the true spatial peak");
    }

    {
        EdgeImageData edgeData = createEmptyEdgeData(cv::Size(260, 180));
        const MatchPose gt = MatchPose::fromDeg(190.0, 90.0, 15.0);
        addLineInterference(edgeData, cv::Point2d(25.0, 15.0), cv::Point2d(25.0, 165.0), 400);
        addInstance(edgeData, model, gt, 80.0);
        ResponseDebugReport report;
        ResponseCandidateGenerator generator(config);
        const std::vector<ResponsePeak> peaks = generator.generatePeaks(model, edgeData, 0, &report);
        printReport("weak_edge_polarity_change", report);
        ok &= expect(hasSpatialPeakNear(peaks, gt, 10.0), "weak polarity-change target should still produce a response peak");
    }

    {
        const cv::Size imageSize(1024, 768);
        EdgeImageData edgeData = createEmptyEdgeData(imageSize);
        const std::vector<MatchPose> gt{
            MatchPose::fromDeg(250.0, 220.0, -15.0),
            MatchPose::fromDeg(760.0, 520.0, 34.0)
        };
        for (const MatchPose& pose : gt) {
            addInstance(edgeData, model, pose);
        }
        CoarseMatchConfig coarseConfig;
        coarseConfig.pyramidLevels = 4;
        ImagePyramid imagePyramid;
        TemplatePyramid templatePyramid;
        ok &= expect(imagePyramid.build(edgeData, coarseConfig.pyramidLevels, coarseConfig), "large image pyramid build should pass");
        ok &= expect(templatePyramid.build(model, coarseConfig.pyramidLevels, coarseConfig.maxTemplatePointsPerLevel),
                     "large template pyramid build should pass");

        OrientationResponseConfig highConfig = config;
        highConfig.maxTemplateResponsePoints = 60;
        ResponseDebugReport report;
        ResponseCandidateGenerator generator(highConfig);
        const int level = 2;
        const std::vector<ResponsePeak> peaks =
            generator.generatePeaks(templatePyramid.level(level), imagePyramid.level(level), level, &report);
        printReport("large_high_level", report);
        ok &= expect(!peaks.empty(), "large high-level response should produce peaks");
        ok &= expect(report.totalPeaksAfterNms <= highConfig.maxTotalPeaks, "peak count should be capped");
    }

    {
        CoarseMatchConfig coarseConfig;
        coarseConfig.pyramidLevels = 2;
        ImagePyramid imagePyramid;
        TemplatePyramid templatePyramid;
        const MatchPose gt = MatchPose::fromDeg(105.0, 85.0, 0.0);
        const EdgeImageData edgeData = EdgeImageData::createFromTemplateAndPose(model, gt, cv::Size(220, 170));
        imagePyramid.build(edgeData, coarseConfig.pyramidLevels, coarseConfig);
        templatePyramid.build(model, coarseConfig.pyramidLevels, coarseConfig.maxTemplatePointsPerLevel);
        ShapeMatchPipelineV2Context context;
        context.templateModel = &model;
        context.edgeData = &edgeData;
        context.imagePyramid = &imagePyramid;
        context.templatePyramid = &templatePyramid;
        context.groundTruth.push_back({"obj_001", gt});
        context.imageName = "response_pipeline_v2_synthetic.png";
        context.templateName = model.templateId;

        ShapeMatchPipelineV2Config pipelineConfig;
        pipelineConfig.candidateGenerationMode = CandidateGenerationMode::OrientationResponse;
        pipelineConfig.pyramidLevel = 0;
        pipelineConfig.responsePipeline.responsePyramidLevel = 0;
        pipelineConfig.responsePipeline.executionMode = PipelineV2ExecutionMode::CandidateOnly;
        pipelineConfig.orientationResponseConfig = config;
        ShapeMatchPipelineV2 pipeline(pipelineConfig);
        const ShapeMatchPipelineV2Result result = pipeline.run(context);
        std::cout << "[ResponseCandidate] pipelineV2 candidates=" << result.candidateCount
                  << " recall=" << result.recall << '\n';
        ok &= expect(result.ok, "PipelineV2 OrientationResponse mode should succeed");
        ok &= expect(result.candidateCount > 0, "PipelineV2 OrientationResponse should produce candidates");
    }

    const std::filesystem::path reportDir = std::filesystem::path("data") / "shape_match" / "reports";
    ok &= expect(std::filesystem::exists(reportDir / "latest_response_peaks.csv"), "response peaks csv should exist");
    ok &= expect(std::filesystem::exists(reportDir / "latest_response_max_xy.png"), "response max image should exist");
    ok &= expect(std::filesystem::exists(reportDir / "latest_response_peaks_overlay.png"), "response overlay image should exist");

    if (ok) {
        std::cout << "[ResponseCandidate] self-test PASSED\n";
    }
    return ok ? 0 : 1;
}

} // namespace ShapeMatch

int main()
{
    return ShapeMatch::runResponseCandidateGeneratorSelfTest();
}
