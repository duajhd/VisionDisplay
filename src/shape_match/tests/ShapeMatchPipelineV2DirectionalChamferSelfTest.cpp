#include "shape_match/coarse/CoarseMatchConfig.h"
#include "shape_match/coarse/ImagePyramid.h"
#include "shape_match/coarse/TemplatePyramid.h"
#include "shape_match/core/EdgeImageData.h"
#include "shape_match/core/ShapeTemplateModel.h"
#include "shape_match/pipeline_v2/ShapeMatchPipelineV2.h"

#include <algorithm>
#include <filesystem>
#include <iostream>

namespace ShapeMatch {

namespace {

bool expect(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "[PipelineV2DirectionalChamfer][FAIL] " << message << '\n';
    }
    return condition;
}

EdgeImageData emptyData(cv::Size size)
{
    EdgeImageData data;
    data.imageSize = size;
    data.edgeMap = cv::Mat::zeros(size, CV_8U);
    data.gradX = cv::Mat::zeros(size, CV_32F);
    data.gradY = cv::Mat::zeros(size, CV_32F);
    data.gradMag = cv::Mat::zeros(size, CV_32F);
    return data;
}

void addInstance(EdgeImageData& dst, const ShapeTemplateModel& model, const MatchPose& pose)
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
        dst.gradMag.at<float>(y, x) = 255.0f;
    }
}

ShapeMatchPipelineV2Result runCase(bool enableChamfer)
{
    const ShapeTemplateModel model = ShapeTemplateModel::createSyntheticRectangle("pipeline_v2_dc_rect", 48.0, 32.0, 18);
    EdgeImageData data = emptyData(cv::Size(360, 260));
    std::vector<GroundTruthInstance> gt{
        {"obj_001", MatchPose::fromDeg(85.0, 70.0, -10.0)},
        {"obj_002", MatchPose::fromDeg(275.0, 70.0, 28.0)},
        {"obj_003", MatchPose::fromDeg(85.0, 195.0, 62.0)},
        {"obj_004", MatchPose::fromDeg(275.0, 195.0, -38.0)}
    };
    for (const GroundTruthInstance& g : gt) {
        addInstance(data, model, g.pose);
    }

    CoarseMatchConfig coarseConfig;
    coarseConfig.pyramidLevels = 1;
    ImagePyramid imagePyramid;
    TemplatePyramid templatePyramid;
    imagePyramid.build(data, coarseConfig.pyramidLevels, coarseConfig);
    templatePyramid.build(model, coarseConfig.pyramidLevels, coarseConfig.maxTemplatePointsPerLevel);

    ShapeMatchPipelineV2Context context;
    context.templateModel = &model;
    context.edgeData = &data;
    context.imagePyramid = &imagePyramid;
    context.templatePyramid = &templatePyramid;
    context.groundTruth = gt;
    context.imageName = "pipeline_v2_directional_chamfer.png";
    context.templateName = model.templateId;

    ShapeMatchPipelineV2Config config;
    config.candidateGenerationMode = CandidateGenerationMode::OrientationResponse;
    config.pyramidLevel = 0;
    config.responsePipeline.responsePyramidLevel = 0;
    config.responsePipeline.executionMode = PipelineV2ExecutionMode::FullCoarseMatch;
    config.responsePipeline.maxResponsePeaks = 300;
    config.responsePipeline.maxVerifiedResponsePeaks = 160;
    config.responsePipeline.maxFinalCandidates = 60;
    config.responsePipeline.targetFinalCandidates = 30;
    config.orientationResponseConfig.thetaBinCount = 36;
    config.orientationResponseConfig.maxTemplateResponsePoints = 80;
    config.orientationResponseConfig.maxTotalPeaks = 300;
    config.orientationResponseConfig.minResponseScore = 0.10;
    config.directionalChamfer.enableDirectionalChamferVerification = enableChamfer;
    config.directionalChamfer.maxCandidatesToVerify = 160;
    config.directionalChamfer.targetCandidatesAfterVerify = 60;
    ShapeMatchPipelineV2 pipeline(config);
    return pipeline.run(context);
}

} // namespace

int main()
{
    bool ok = true;
    const ShapeMatchPipelineV2Result r3 = runCase(false);
    const ShapeMatchPipelineV2Result r4 = runCase(true);
    std::cout << "[PipelineV2DirectionalChamfer] r3 selected=" << r3.selectedCandidateCount
              << " finalMs=" << r3.finalRankerMs
              << " recall=" << r3.recall << '\n';
    std::cout << "[PipelineV2DirectionalChamfer] r4 chamferIn=" << r4.directionalChamferInputCount
              << " chamferAccepted=" << r4.directionalChamferAcceptedCount
              << " selected=" << r4.selectedCandidateCount
              << " chamferMs=" << r4.directionalChamferTimeMs
              << " finalMs=" << r4.finalRankerMs
              << " recall=" << r4.recall << '\n';
    ok &= expect(r4.ok, "R4 PipelineV2 should succeed");
    ok &= expect(!r4.usedOldLocalRefine, "R4 should not use old local refine");
    ok &= expect(r4.oldLevel0RawChildren == 0, "R4 level0 raw children should be zero");
    ok &= expect(r4.directionalChamferInputCount > 0, "R4 should run directional chamfer");
    ok &= expect(r4.directionalChamferAcceptedCount <= r4.directionalChamferInputCount, "chamfer should not increase candidates");
    ok &= expect(r4.selectedCandidateCount <= r3.selectedCandidateCount, "R4 selected candidates should not exceed R3");
    ok &= expect(r4.recall + 0.25 >= r3.recall, "R4 recall should not drop heavily");
    const std::filesystem::path reportDir = std::filesystem::path("data") / "shape_match" / "reports";
    ok &= expect(std::filesystem::exists(reportDir / "latest_directional_chamfer_summary.txt"), "directional chamfer summary should exist");
    ok &= expect(std::filesystem::exists(reportDir / "latest_pipeline_v2_directional_chamfer_vs_r3.csv"), "R3 vs R4 csv should exist");
    if (ok) {
        std::cout << "[PipelineV2DirectionalChamfer] self-test PASSED\n";
    }
    return ok ? 0 : 1;
}

} // namespace ShapeMatch

int main()
{
    return ShapeMatch::main();
}
