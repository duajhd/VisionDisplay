#include "shape_match/coarse/CoarseMatchConfig.h"
#include "shape_match/coarse/ImagePyramid.h"
#include "shape_match/coarse/OrientationResponseMapBuilder.h"
#include "shape_match/coarse/ResponseMapEvaluator.h"
#include "shape_match/coarse/ResponseTemplate.h"
#include "shape_match/coarse/TemplatePyramid.h"
#include "shape_match/core/EdgeImageData.h"
#include "shape_match/core/ShapeTemplateModel.h"
#include "shape_match/pipeline_v2/DirectionalFieldCache.h"
#include "shape_match/pipeline_v2/DirectionalChamferVerifier.h"
#include "shape_match/pipeline_v2/DirectionalDistanceFieldBuilder.h"
#include "shape_match/pipeline_v2/ParallelDirectionalChamferVerifier.h"
#include "shape_match/pipeline_v2/ParallelThetaResponseEvaluator.h"
#include "shape_match/pipeline_v2/RuntimeBufferPool.h"
#include "shape_match/pipeline_v2/ShapeMatchPipelineV2.h"
#include "shape_match/pipeline_v2/TemplateRuntimeCache.h"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>

namespace ShapeMatch {

namespace {

bool expect(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "[PipelineV2Benchmark][FAIL] " << message << '\n';
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

struct Fixture
{
    ShapeTemplateModel model;
    EdgeImageData edgeData;
    ImagePyramid imagePyramid;
    TemplatePyramid templatePyramid;
    std::vector<GroundTruthInstance> gt;
};

Fixture makeFixture()
{
    Fixture f;
    f.model = ShapeTemplateModel::createSyntheticRectangle("pipeline_v2_benchmark_rect", 48.0, 32.0, 18);
    f.edgeData = emptyData(cv::Size(360, 260));
    f.gt = {
        {"obj_001", MatchPose::fromDeg(85.0, 70.0, -10.0)},
        {"obj_002", MatchPose::fromDeg(275.0, 70.0, 28.0)},
        {"obj_003", MatchPose::fromDeg(85.0, 195.0, 62.0)},
        {"obj_004", MatchPose::fromDeg(275.0, 195.0, -38.0)}
    };
    for (const GroundTruthInstance& g : f.gt) {
        addInstance(f.edgeData, f.model, g.pose);
    }
    CoarseMatchConfig coarseConfig;
    coarseConfig.pyramidLevels = 1;
    f.imagePyramid.build(f.edgeData, coarseConfig.pyramidLevels, coarseConfig);
    f.templatePyramid.build(f.model, coarseConfig.pyramidLevels, coarseConfig.maxTemplatePointsPerLevel);
    return f;
}

ShapeMatchPipelineV2Config baseConfig(ShapeMatchRuntimeMode mode)
{
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
    config.directionalChamfer.maxCandidatesToVerify = 160;
    config.directionalChamfer.targetCandidatesAfterVerify = 60;
    config.production.runtimeMode = mode;
    config.production.numWorkerThreads = 2;
    return config;
}

ShapeMatchPipelineV2Result runPipeline(const Fixture& f, ShapeMatchRuntimeMode mode)
{
    ShapeMatchPipelineV2Context context;
    context.templateModel = &f.model;
    context.edgeData = &f.edgeData;
    context.imagePyramid = &f.imagePyramid;
    context.templatePyramid = &f.templatePyramid;
    context.groundTruth = f.gt;
    context.imageName = "pipeline_v2_benchmark.png";
    context.templateName = f.model.templateId;
    ShapeMatchPipelineV2 pipeline(baseConfig(mode));
    return pipeline.run(context);
}

void writeBenchmarkReports(const std::vector<ShapeMatchPipelineV2Result>& runs)
{
    const std::filesystem::path dir = std::filesystem::path("data") / "shape_match" / "reports";
    std::filesystem::create_directories(dir);
    {
        std::ofstream out(dir / "latest_pipeline_v2_benchmark.csv");
        out << "run_id,mode,total_ms,response_ms,theta_eval_ms,peak_extract_ms,fast_verify_ms,directional_field_build_ms,directional_chamfer_ms,final_ranker_ms,cache_hit_response_template,cache_hit_segment_template,cache_hit_directional_field,mat_allocated,mat_reused,vector_allocated,vector_reused,candidate_count,recall\n";
        for (size_t i = 0; i < runs.size(); ++i) {
            const auto& r = runs[i];
            out << i << ",Benchmark,"
                << r.totalTimeMs << ','
                << r.responseGenerationMs << ",0,0,"
                << r.responseVerificationMs << ','
                << r.directionalChamferFieldBuildTimeMs << ','
                << r.directionalChamferTimeMs << ','
                << r.finalRankerMs << ','
                << r.responseTemplateCacheHits << ','
                << r.segmentTemplateCacheHits << ','
                << r.directionalFieldCacheHits << ','
                << r.matAllocatedCount << ','
                << r.matReusedCount << ','
                << r.vectorAllocatedCount << ','
                << r.vectorReusedCount << ','
                << r.selectedCandidateCount << ','
                << r.recall << '\n';
        }
    }
    std::vector<double> values;
    for (const auto& r : runs) {
        values.push_back(r.totalTimeMs);
    }
    std::sort(values.begin(), values.end());
    const double avg = values.empty() ? 0.0 : std::accumulate(values.begin(), values.end(), 0.0) / values.size();
    auto pct = [&](double p) {
        if (values.empty()) {
            return 0.0;
        }
        const size_t idx = std::min(values.size() - 1, static_cast<size_t>(p * (values.size() - 1)));
        return values[idx];
    };
    {
        std::ofstream out(dir / "latest_pipeline_v2_benchmark_summary.txt");
        out << "PipelineV2 Benchmark Summary\n";
        out << "runs: " << runs.size() << '\n';
        out << "avgMs: " << avg << '\n';
        out << "p50Ms: " << pct(0.50) << '\n';
        out << "p90Ms: " << pct(0.90) << '\n';
        out << "p99Ms: " << pct(0.99) << '\n';
        if (!runs.empty()) {
            out << "lastRecall: " << runs.back().recall << '\n';
            out << "lastDirectionalFieldCacheHits: " << runs.back().directionalFieldCacheHits << '\n';
        }
    }
}

} // namespace

int main()
{
    bool ok = true;
    RuntimeBufferPool pool;
    cv::Mat& a = pool.acquireFloatMat(cv::Size(32, 24), "response");
    (void)a;
    pool.resetForRun();
    cv::Mat& b = pool.acquireFloatMat(cv::Size(32, 24), "response");
    (void)b;
    auto& v1 = pool.acquireVector<int>("peaks", 64);
    v1.push_back(1);
    auto& v2 = pool.acquireVector<int>("peaks", 64);
    ok &= expect(pool.reusedMatCount() > 0, "RuntimeBufferPool should reuse mats");
    ok &= expect(v2.empty() && pool.reusedVectorCount() > 0, "RuntimeBufferPool should reuse vectors");

    Fixture f = makeFixture();
    OrientationResponseConfig responseConfig = baseConfig(ShapeMatchRuntimeMode::Debug).orientationResponseConfig;
    DirectionalChamferConfig chamferConfig = baseConfig(ShapeMatchRuntimeMode::Debug).directionalChamfer;
    TemplateRuntimeCache templateCache;
    const ResponseTemplate& rt1 = templateCache.getOrBuildResponseTemplate(f.model, 0, responseConfig);
    const ResponseTemplate& rt2 = templateCache.getOrBuildResponseTemplate(f.model, 0, responseConfig);
    const SegmentTemplate& st1 = templateCache.getOrBuildSegmentTemplate(f.model, 0, chamferConfig);
    const SegmentTemplate& st2 = templateCache.getOrBuildSegmentTemplate(f.model, 0, chamferConfig);
    ok &= expect(templateCache.responseTemplateHitCount() == 1 && rt1.size() == rt2.size(), "ResponseTemplate cache should hit");
    ok &= expect(templateCache.segmentTemplateHitCount() == 1 && st1.pointCount() == st2.pointCount(), "SegmentTemplate cache should hit");

    DirectionalFieldCache fieldCache;
    const DirectionalDistanceField& df1 = fieldCache.getOrBuildDirectionalField(f.edgeData, 0, chamferConfig);
    const DirectionalDistanceField& df2 = fieldCache.getOrBuildDirectionalField(f.edgeData, 0, chamferConfig);
    ok &= expect(fieldCache.directionalFieldHitCount() == 1 && df1.valid && df2.valid, "DirectionalField cache should hit");

    const OrientationResponseMap responseMap = OrientationResponseMapBuilder().build(f.edgeData, 0, responseConfig);
    ResponseDebugReport serialReport;
    PipelineV2ProductionConfig serialProd;
    serialProd.enableParallelThetaEvaluation = false;
    std::vector<ResponsePeak> serialPeaks =
        ParallelThetaResponseEvaluator(serialProd).evaluateAllTheta(responseMap, rt1, responseConfig, &serialReport, nullptr);
    ResponseDebugReport parallelReport;
    PipelineV2ProductionConfig parallelProd;
    parallelProd.enableParallelThetaEvaluation = true;
    parallelProd.numWorkerThreads = 2;
    std::vector<ResponsePeak> parallelPeaks =
        ParallelThetaResponseEvaluator(parallelProd).evaluateAllTheta(responseMap, rt1, responseConfig, &parallelReport, nullptr);
    ok &= expect(!serialPeaks.empty() && !parallelPeaks.empty(), "ParallelTheta should produce peaks");

    std::vector<VerifiedResponsePeak> verified;
    for (size_t i = 0; i < std::min<size_t>(serialPeaks.size(), 40); ++i) {
        VerifiedResponsePeak item;
        item.peak = serialPeaks[i];
        item.pose = serialPeaks[i].pose;
        item.accepted = true;
        item.responseScore = serialPeaks[i].responseScore;
        verified.push_back(item);
    }
    DirectionalChamferVerifier verifier(chamferConfig);
    std::vector<VerifiedResponsePeak> serialChamfer = verifier.verifyPeaks(verified, st1, df1);
    int pruned = 0;
    std::vector<VerifiedResponsePeak> parallelChamfer =
        ParallelDirectionalChamferVerifier(parallelProd).verify(verified, st1, df1, verifier, chamferConfig.targetCandidatesAfterVerify, &pruned);
    ok &= expect(!serialChamfer.empty() && !parallelChamfer.empty(), "Parallel chamfer should produce candidates");

    ShapeMatchPipelineV2Result debugResult = runPipeline(f, ShapeMatchRuntimeMode::Debug);
    ShapeMatchPipelineV2Result productionResult = runPipeline(f, ShapeMatchRuntimeMode::Production);
    ok &= expect(debugResult.ok && productionResult.ok, "Debug and Production pipeline runs should succeed");
    ok &= expect(!productionResult.usedOldLocalRefine && productionResult.oldLevel0RawChildren == 0, "Production should not use old local refine");
    ok &= expect(productionResult.recall + 0.01 >= debugResult.recall, "Production recall should not regress");

    std::vector<ShapeMatchPipelineV2Result> benchmarkRuns;
    for (int i = 0; i < 5; ++i) {
        benchmarkRuns.push_back(runPipeline(f, ShapeMatchRuntimeMode::Benchmark));
    }
    writeBenchmarkReports(benchmarkRuns);
    ok &= expect(std::filesystem::exists(std::filesystem::path("data") / "shape_match" / "reports" / "latest_pipeline_v2_benchmark.csv"),
                 "Benchmark CSV should exist");

    std::cout << "[PipelineV2Benchmark] debugMs=" << debugResult.totalTimeMs
              << " productionMs=" << productionResult.totalTimeMs
              << " productionRecall=" << productionResult.recall
              << " fieldCacheHits=" << productionResult.directionalFieldCacheHits << '\n';
    if (ok) {
        std::cout << "[PipelineV2Benchmark] self-test PASSED\n";
    }
    return ok ? 0 : 1;
}

} // namespace ShapeMatch

int main()
{
    return ShapeMatch::main();
}
