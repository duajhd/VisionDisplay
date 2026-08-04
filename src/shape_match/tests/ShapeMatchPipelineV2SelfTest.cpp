#include "shape_match/coarse/CoarseMatchConfig.h"
#include "shape_match/coarse/ImagePyramid.h"
#include "shape_match/coarse/TemplatePyramid.h"
#include "shape_match/core/EdgeImageData.h"
#include "shape_match/core/ShapeTemplateModel.h"
#include "shape_match/pipeline_v2/ShapeMatchPipelineV2.h"

#include <filesystem>
#include <iostream>
#include <string>

namespace ShapeMatch {

namespace {

bool expect(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "[PipelineV2][FAIL] " << message << '\n';
    }
    return condition;
}

bool existsNonEmpty(const std::filesystem::path& path)
{
    std::error_code ec;
    return std::filesystem::exists(path, ec) && std::filesystem::file_size(path, ec) > 0;
}

} // namespace

bool runShapeMatchPipelineV2SelfTest()
{
    bool ok = true;

    const ShapeTemplateModel model =
        ShapeTemplateModel::createSyntheticRectangle("pipeline_v2_rect", 96.0, 64.0, 28);
    const MatchPose gtPose = MatchPose::fromDeg(320.0, 240.0, 17.0);
    const EdgeImageData edgeData = EdgeImageData::createFromTemplateAndPose(model, gtPose, cv::Size(640, 480));

    CoarseMatchConfig coarseConfig;
    coarseConfig.pyramidLevels = 4;

    ImagePyramid imagePyramid;
    TemplatePyramid templatePyramid;
    ok &= expect(imagePyramid.build(edgeData, coarseConfig.pyramidLevels, coarseConfig),
                 "ImagePyramid build should succeed");
    ok &= expect(templatePyramid.build(model, coarseConfig.pyramidLevels, coarseConfig.maxTemplatePointsPerLevel),
                 "TemplatePyramid build should succeed");

    ShapeMatchPipelineV2Context context;
    context.templateModel = &model;
    context.edgeData = &edgeData;
    context.imagePyramid = &imagePyramid;
    context.templatePyramid = &templatePyramid;
    context.groundTruth.push_back({"obj_001", gtPose});
    context.imageName = "synthetic_pipeline_v2.png";
    context.templateName = model.templateId;

    const std::filesystem::path reportDir = std::filesystem::path("data") / "shape_match" / "reports";

    {
        ShapeMatchPipelineV2Config config;
        config.candidateGenerationMode = CandidateGenerationMode::OrientationResponse;
        config.pyramidLevel = 0;
        config.responsePipeline.responsePyramidLevel = 0;
        config.responsePipeline.executionMode = PipelineV2ExecutionMode::CandidateOnly;
        config.enableDebugReport = true;
        config.reportDir = reportDir;
        config.orientationResponseConfig.thetaBinCount = 36;
        config.orientationResponseConfig.maxTemplateResponsePoints = 80;
        config.orientationResponseConfig.maxTotalPeaks = 200;
        config.orientationResponseConfig.minResponseScore = 0.10;

        ShapeMatchPipelineV2 pipeline(config);
        const ShapeMatchPipelineV2Result result = pipeline.run(context);

        std::cout << "[PipelineV2] mode=OrientationResponse ok=" << result.ok
                  << " candidates=" << result.candidateCount
                  << " recall=" << result.recall << '\n';

        ok &= expect(result.ok, "OrientationResponse mode should succeed after R2");
        ok &= expect(result.candidateCount > 0, "OrientationResponse mode should produce candidates");
    }

    {
        ShapeMatchPipelineV2Config config;
        config.candidateGenerationMode = CandidateGenerationMode::OldVoting;
        config.pyramidLevel = 0;
        config.maxCandidates = 200;
        config.enableDebugReport = true;
        config.reportDir = reportDir;

        ShapeMatchPipelineV2 pipeline(config);
        const ShapeMatchPipelineV2Result result = pipeline.run(context);

        std::cout << "[PipelineV2] mode=OldVoting ok=" << result.ok
                  << " candidates=" << result.candidateCount
                  << " genMs=" << result.candidateGenerationMs
                  << " recall=" << result.recall << '\n';

        ok &= expect(result.ok, "OldVoting mode should succeed");
        ok &= expect(!result.candidates.empty(), "OldVoting mode should produce candidates");
        ok &= expect(result.candidateCount > 0, "OldVoting candidateCount should be positive");
        ok &= expect(existsNonEmpty(reportDir / "latest_pipeline_v2_summary.txt"),
                     "pipeline v2 summary report should be generated");
        ok &= expect(existsNonEmpty(reportDir / "latest_pipeline_v2_candidates.csv"),
                     "pipeline v2 candidates csv should be generated");
    }

    std::cout << "[PipelineV2] reports_dir=" << reportDir.string() << '\n';
    if (ok) {
        std::cout << "[PipelineV2] self-test PASSED\n";
    }
    return ok;
}

} // namespace ShapeMatch

int main()
{
    return ShapeMatch::runShapeMatchPipelineV2SelfTest() ? 0 : 1;
}
