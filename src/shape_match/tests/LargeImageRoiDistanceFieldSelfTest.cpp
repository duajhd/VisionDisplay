#include "shape_match/coarse/DistanceFieldBuilder.h"
#include "shape_match/coarse/ImagePyramid.h"
#include "shape_match/coarse/RoiDistanceFieldBuilder.h"
#include "shape_match/evaluation/CandidateRanker.h"

#include <chrono>
#include <iostream>

namespace ShapeMatch {
namespace {

bool expect(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "[LargeImageRoiDistanceField][FAIL] " << message << '\n';
        return false;
    }
    return true;
}

ShapeTemplateModel makeTemplate()
{
    ShapeTemplateModel model = ShapeTemplateModel::createSyntheticRectangle("roi_template", 80.0, 40.0, 30);
    model.computeBoundingBox();
    return model;
}

EdgeImageData makeEdgeData(cv::Size size, const MatchPose& pose)
{
    EdgeImageData data = EdgeImageData::createFromTemplateAndPose(makeTemplate(), pose, size);
    return data;
}

bool testRoiDistanceFieldCorrectness()
{
    EdgeImageData data = makeEdgeData(cv::Size(400, 300), MatchPose::fromDeg(200.0, 150.0, 0.0));
    RoiDistanceFieldBuilder builder;
    CoarseMatchConfig::RoiDistanceFieldConfig config;
    config.maxTotalRoiPixelsForDistanceField = 200000;
    const RoiDistanceFieldSet set = builder.buildForRegions(data, {cv::Rect(130, 90, 140, 120)}, 0, 1.0, config);

    EdgeQueryContext ctx;
    ctx.fullEdgeData = &data;
    ctx.roiFields = &set;
    ctx.allowFullField = false;
    cv::Point2d matched;
    cv::Point2d normal;
    double mag = 0.0;
    double dist = 0.0;
    std::string mode;
    const bool found = queryNearestEdge(ctx, 0, cv::Point2d(200.0, 130.0), 20.0, matched, normal, mag, dist, &mode);
    bool ok = true;
    ok &= expect(set.size() == 1, "ROI field should be built");
    ok &= expect(found, "ROI query should find edge");
    ok &= expect(mode == "roi_distance_field", "query mode should be ROI distance field");
    ok &= expect(std::abs(dist) <= 2.0, "edge point distance should be near zero");
    return ok;
}

bool testFullVsRoiConsistency()
{
    EdgeImageData full = makeEdgeData(cv::Size(400, 300), MatchPose::fromDeg(200.0, 150.0, 0.0));
    CoarseMatchConfig fullConfig;
    DistanceFieldBuilder().build(full, fullConfig);
    RoiDistanceFieldBuilder builder;
    CoarseMatchConfig::RoiDistanceFieldConfig roiConfig;
    const RoiDistanceFieldSet set = builder.buildForRegions(full, {cv::Rect(120, 80, 180, 140)}, 0, 1.0, roiConfig);

    EdgeQueryContext roiCtx;
    roiCtx.fullEdgeData = &full;
    roiCtx.roiFields = &set;
    roiCtx.allowFullField = false;

    bool ok = true;
    for (const cv::Point2d p : {cv::Point2d(200, 130), cv::Point2d(240, 150), cv::Point2d(200, 170)}) {
        cv::Point2d fm, rm, fn, rn;
        double fmag = 0.0, rmag = 0.0, fd = 0.0, rd = 0.0;
        const bool ff = full.findNearestEdgeFast(p, 30.0, fm, fn, fmag, fd);
        const bool rf = queryNearestEdge(roiCtx, 0, p, 30.0, rm, rn, rmag, rd, nullptr);
        ok &= expect(ff && rf, "full and ROI should both find edge");
        ok &= expect(std::abs(fd - rd) <= 1.5, "full and ROI distances should be close");
        ok &= expect(norm(fm - rm) <= 2.0, "full and ROI nearest points should be close");
    }
    return ok;
}

bool testSkipLevel0FullField()
{
    EdgeImageData data = makeEdgeData(cv::Size(3000, 3000), MatchPose::fromDeg(1500.0, 1500.0, 0.0));
    CoarseMatchConfig config;
    config.pyramidLevels = 3;
    config.roiDistanceField.enableRoiDistanceField = true;
    config.roiDistanceField.largeImageMinPixelsForRoiDistanceField = 1000;
    config.roiDistanceField.buildFullDistanceFieldForLevel0 = false;
    ImagePyramid pyramid;
    const bool built = pyramid.build(data, 3, config);
    bool ok = true;
    ok &= expect(built, "pyramid should build");
    ok &= expect(!pyramid.level(0).hasDistanceField, "level0 full distance field should be skipped");
    ok &= expect(pyramid.skippedFullDistanceField(0), "level0 skip flag should be true");
    ok &= expect(pyramid.level(1).hasDistanceField, "level1 full distance field should remain available");
    return ok;
}

bool testFinalRankerUsesRoiField()
{
    const ShapeTemplateModel model = makeTemplate();
    EdgeImageData data = EdgeImageData::createFromTemplateAndPose(model, MatchPose::fromDeg(220.0, 160.0, 0.0), cv::Size(500, 400));
    RoiDistanceFieldBuilder builder;
    CoarseMatchConfig::RoiDistanceFieldConfig roiConfig;
    const RoiDistanceFieldSet set = builder.buildForRegions(data, {cv::Rect(140, 90, 180, 140)}, 0, 1.0, roiConfig);

    EdgeQueryContext ctx;
    ctx.fullEdgeData = &data;
    ctx.roiFields = &set;
    ctx.allowFullField = false;
    ShapeMatchEvalConfig evalConfig;
    evalConfig.topK = 3;
    evalConfig.minValidPoints = 1;
    CandidateRanker ranker(evalConfig);
    FinalRankerProfile profile;
    const std::vector<ScoredCandidate> ranked = ranker.rank({MatchPose::fromDeg(220.0, 160.0, 0.0),
                                                             MatchPose::fromDeg(260.0, 160.0, 0.0)},
                                                            model,
                                                            ctx,
                                                            nullptr,
                                                            &profile);
    bool ok = true;
    ok &= expect(!ranked.empty(), "ranker should return candidates");
    ok &= expect(profile.roiDistanceFieldQueryCount > 0, "final ranker should query ROI distance field");
    ok &= expect(profile.linearScanFallbackCount == 0, "final ranker should not linear scan");
    return ok;
}

bool testSyntheticLargeImageSmoke()
{
    const ShapeTemplateModel model = makeTemplate();
    EdgeImageData data = EdgeImageData::createFromTemplateAndPose(model, MatchPose::fromDeg(2500.0, 1800.0, 0.0), cv::Size(5472, 3648));
    CoarseMatchConfig config;
    config.pyramidLevels = 4;
    config.roiDistanceField.enableRoiDistanceField = true;
    config.roiDistanceField.largeImageMinPixelsForRoiDistanceField = 1000;
    config.roiDistanceField.buildFullDistanceFieldForLevel0 = false;
    ImagePyramid pyramid;
    const auto t0 = std::chrono::steady_clock::now();
    const bool built = pyramid.build(data, 4, config);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    std::cout << "[LargeImageRoiDistanceField] synthetic_pyramid_ms=" << ms
              << " level0Skipped=" << (built && pyramid.skippedFullDistanceField(0))
              << " fullDfMs=" << (built ? pyramid.totalDistanceFieldBuildTimeMs() : 0.0) << '\n';
    return expect(built && pyramid.skippedFullDistanceField(0), "synthetic large image should skip level0 full field");
}

} // namespace
} // namespace ShapeMatch

int main()
{
    bool ok = true;
    ok &= ShapeMatch::testRoiDistanceFieldCorrectness();
    ok &= ShapeMatch::testFullVsRoiConsistency();
    ok &= ShapeMatch::testSkipLevel0FullField();
    ok &= ShapeMatch::testFinalRankerUsesRoiField();
    ok &= ShapeMatch::testSyntheticLargeImageSmoke();
    if (ok) {
        std::cout << "[LargeImageRoiDistanceField] self-test PASSED\n";
    }
    return ok ? 0 : 1;
}
