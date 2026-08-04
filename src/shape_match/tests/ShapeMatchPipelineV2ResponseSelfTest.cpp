#include "shape_match/coarse/CoarseMatchConfig.h"
#include "shape_match/coarse/ImagePyramid.h"
#include "shape_match/coarse/TemplatePyramid.h"
#include "shape_match/core/EdgeImageData.h"
#include "shape_match/core/ShapeTemplateModel.h"
#include "shape_match/pipeline_v2/ShapeMatchPipelineV2.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace ShapeMatch {

namespace {

bool expect(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "[PipelineV2Response][FAIL] " << message << '\n';
    }
    return condition;
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

ShapeMatchPipelineV2Config pipelineConfig(int level)
{
    ShapeMatchPipelineV2Config config;
    config.candidateGenerationMode = CandidateGenerationMode::OrientationResponse;
    config.pyramidLevel = level;
    config.responsePipeline.responsePyramidLevel = level;
    config.responsePipeline.executionMode = PipelineV2ExecutionMode::FullCoarseMatch;
    config.responsePipeline.maxResponsePeaks = 300;
    config.responsePipeline.maxVerifiedResponsePeaks = 120;
    config.responsePipeline.maxFinalCandidates = 50;
    config.responsePipeline.targetFinalCandidates = 20;
    config.responsePipeline.regionPaddingPxLevel0 = 24;
    config.responsePipeline.enableFinalRanker = true;
    config.orientationResponseConfig.thetaBinCount = 36;
    config.orientationResponseConfig.maxTemplateResponsePoints = 80;
    config.orientationResponseConfig.maxTotalPeaks = 300;
    config.orientationResponseConfig.maxPeaksPerTheta = 80;
    config.orientationResponseConfig.minResponseScore = 0.10;
    config.orientationResponseConfig.spatialSpreadRadiusPx = 3;
    return config;
}

ShapeMatchPipelineV2Result runPipeline(const ShapeTemplateModel& model,
                                       const EdgeImageData& edgeData,
                                       const std::vector<GroundTruthInstance>& gt,
                                       int level)
{
    CoarseMatchConfig coarseConfig;
    coarseConfig.pyramidLevels = std::max(1, level + 1);
    ImagePyramid imagePyramid;
    TemplatePyramid templatePyramid;
    imagePyramid.build(edgeData, coarseConfig.pyramidLevels, coarseConfig);
    templatePyramid.build(model, coarseConfig.pyramidLevels, coarseConfig.maxTemplatePointsPerLevel);

    ShapeMatchPipelineV2Context context;
    context.templateModel = &model;
    context.edgeData = &edgeData;
    context.imagePyramid = &imagePyramid;
    context.templatePyramid = &templatePyramid;
    context.groundTruth = gt;
    context.imageName = "synthetic_pipeline_v2_response.png";
    context.templateName = model.templateId;

    ShapeMatchPipelineV2 pipeline(pipelineConfig(level));
    return pipeline.run(context);
}

bool checkCommon(const ShapeMatchPipelineV2Result& result, const char* caseName)
{
    std::cout << "[PipelineV2Response] case=" << caseName
              << " ok=" << result.ok
              << " peaks=" << result.responsePeakCount
              << " verified=" << result.verifiedResponsePeakCount
              << " selected=" << result.selectedCandidateCount
              << " regions=" << result.candidateRegionCount
              << " finalMs=" << result.finalRankerMs
              << " totalMs=" << result.totalTimeMs
              << " recall=" << result.recall << '\n';
    bool ok = true;
    ok &= expect(result.ok, std::string(caseName) + " should succeed");
    ok &= expect(!result.usedOldLocalRefine, std::string(caseName) + " should not use old local refine");
    ok &= expect(!result.usedOldBeamPropagation, std::string(caseName) + " should not use old beam propagation");
    ok &= expect(result.oldLevel0RawChildren == 0, std::string(caseName) + " should have zero level0 raw children");
    ok &= expect(result.selectedCandidateCount <= 50, std::string(caseName) + " selected candidates should be capped");
    return ok;
}

} // namespace

int runShapeMatchPipelineV2ResponseSelfTest()
{
    bool ok = true;
    const ShapeTemplateModel model = ShapeTemplateModel::createSyntheticRectangle("pipeline_v2_response_rect", 48.0, 32.0, 18);

    {
        const MatchPose gt = MatchPose::fromDeg(105.0, 85.0, 0.0);
        const EdgeImageData edgeData = EdgeImageData::createFromTemplateAndPose(model, gt, cv::Size(220, 170));
        const ShapeMatchPipelineV2Result result = runPipeline(model, edgeData, {{"obj_001", gt}}, 0);
        ok &= checkCommon(result, "single_no_rotation");
        ok &= expect(result.top10Hit || result.recall > 0.0, "single no-rotation should hit GT");
    }

    {
        const MatchPose gt = MatchPose::fromDeg(110.0, 90.0, 37.0);
        const EdgeImageData edgeData = EdgeImageData::createFromTemplateAndPose(model, gt, cv::Size(230, 180));
        const ShapeMatchPipelineV2Result result = runPipeline(model, edgeData, {{"obj_001", gt}}, 0);
        ok &= checkCommon(result, "single_rotation");
        ok &= expect(result.top10Hit || result.recall > 0.0, "single rotation should hit GT");
    }

    {
        const cv::Size imageSize(360, 260);
        EdgeImageData edgeData = createEmptyEdgeData(imageSize);
        const std::vector<GroundTruthInstance> gt{
            {"obj_001", MatchPose::fromDeg(85.0, 70.0, -10.0)},
            {"obj_002", MatchPose::fromDeg(275.0, 70.0, 28.0)},
            {"obj_003", MatchPose::fromDeg(85.0, 195.0, 62.0)},
            {"obj_004", MatchPose::fromDeg(275.0, 195.0, -38.0)}
        };
        for (const GroundTruthInstance& g : gt) {
            addInstance(edgeData, model, g.pose);
        }
        const ShapeMatchPipelineV2Result result = runPipeline(model, edgeData, gt, 0);
        ok &= checkCommon(result, "four_targets");
        ok &= expect(result.recall >= 0.5, "four-target recall should be nonzero and cover multiple targets");
    }

    {
        const MatchPose gt = MatchPose::fromDeg(125.0, 95.0, -22.0);
        EdgeImageData edgeData = EdgeImageData::createFromTemplateAndPose(model, gt, cv::Size(260, 190));
        addLineInterference(edgeData, cv::Point2d(20.0, 170.0), cv::Point2d(240.0, 20.0), 500);
        const ShapeMatchPipelineV2Result result = runPipeline(model, edgeData, {{"obj_001", gt}}, 0);
        ok &= checkCommon(result, "strong_line_interference");
        ok &= expect(result.recall > 0.0 || result.top10Hit, "strong-line case should not completely fail");
    }

    {
        const cv::Size imageSize(1024, 768);
        EdgeImageData edgeData = createEmptyEdgeData(imageSize);
        const std::vector<GroundTruthInstance> gt{
            {"obj_001", MatchPose::fromDeg(250.0, 220.0, -15.0)},
            {"obj_002", MatchPose::fromDeg(760.0, 520.0, 34.0)}
        };
        for (const GroundTruthInstance& g : gt) {
            addInstance(edgeData, model, g.pose);
        }
        const ShapeMatchPipelineV2Result result = runPipeline(model, edgeData, gt, 2);
        ok &= checkCommon(result, "large_high_level");
        ok &= expect(result.responsePeakCount > 0, "large high-level response should produce peaks");
    }

    const std::filesystem::path reportDir = std::filesystem::path("data") / "shape_match" / "reports";
    ok &= expect(std::filesystem::exists(reportDir / "latest_pipeline_v2_response_summary.txt"),
                 "PipelineV2 response summary should exist");
    ok &= expect(std::filesystem::exists(reportDir / "latest_pipeline_v2_response_profile.json"),
                 "PipelineV2 response profile json should exist");
    ok &= expect(std::filesystem::exists(reportDir / "latest_pipeline_v2_response_candidates.csv"),
                 "PipelineV2 response candidates csv should exist");
    ok &= expect(std::filesystem::exists(reportDir / "latest_pipeline_v2_response_regions.csv"),
                 "PipelineV2 response regions csv should exist");
    ok &= expect(std::filesystem::exists(reportDir / "latest_pipeline_v2_vs_old_pipeline.csv"),
                 "PipelineV2 vs old csv should exist");

    if (ok) {
        std::cout << "[PipelineV2Response] self-test PASSED\n";
    }
    return ok ? 0 : 1;
}

} // namespace ShapeMatch

int main()
{
    return ShapeMatch::runShapeMatchPipelineV2ResponseSelfTest();
}
