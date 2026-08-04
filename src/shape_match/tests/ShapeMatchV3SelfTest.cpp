#include "shape_match/pipeline_v3/AngleViewBuilderV3.h"
#include "shape_match/pipeline_v3/CoarseSearchV3.h"
#include "shape_match/pipeline_v3/ResponseMapBuilderV3.h"
#include "shape_match/pipeline_v3/ScoreKernelV3.h"
#include "shape_match/pipeline_v3/ShapeMatchV3Diagnostic.h"
#include "shape_match/pipeline_v3/ShapeMatcherV3.h"
#include "shape_match/pipeline_v3/PoseRefinerV3.h"

#include <opencv2/imgproc.hpp>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <random>

namespace {
bool expect(bool value, const char* message)
{
    if (!value) std::cerr << "[ShapeMatchV3][FAIL] " << message << '\n';
    return value;
}

bool kernelTest()
{
    std::mt19937 rng(1234567);
    std::uniform_int_distribution<int> byteDist(0, 255);
    bool ok = true;
    for (int iteration = 0; iteration < 100; ++iteration) {
        ShapeMatch::ResponseMapV3 response;
        response.width = 128; response.height = 80;
        for (int bin = 0; bin < ShapeMatch::kOrientationBinCountV3; ++bin) {
            cv::Mat& map = response.binMaps[static_cast<size_t>(bin)];
            map.create(response.height, response.width, CV_8UC1);
            for (int y = 0; y < map.rows; ++y)
                for (int x = 0; x < map.cols; ++x) map.at<std::uint8_t>(y, x) = byteDist(rng);
            response.binData[static_cast<size_t>(bin)] = map.ptr<std::uint8_t>();
        }
        response.stride = static_cast<int>(response.binMaps[0].step[0]);
        ShapeMatch::ShapeModelV3 model;
        for (int i = 0; i < 300; ++i) {
            ShapeMatch::ShapePointV3 p;
            p.x = static_cast<float>((i * 17) % 31 - 15);
            p.y = static_cast<float>((i * 29) % 25 - 12);
            p.quality = static_cast<float>(300 - i);
            p.gradientAngleRadians = static_cast<float>((i % ShapeMatch::kOrientationBinCountV3)
                                                        * 2.0 * ShapeMatch::kPi
                                                        / ShapeMatch::kOrientationBinCountV3);
            p.weight = static_cast<std::uint8_t>(1 + i % 7);
            model.points.push_back(p);
        }
        ShapeMatch::ShapeSearchParametersV3 search;
        search.safetyMode = static_cast<ShapeMatch::SearchSafetyV3>(iteration % 3);
        const auto views = ShapeMatch::AngleViewBuilderV3().build(
            model, 1.0f, response.stride, -0.4f, 0.8f, 0.2f, search);
        const auto& view = views[static_cast<size_t>(iteration % views.size())];
        const std::uint32_t threshold = static_cast<std::uint32_t>(
            (0.25 + 0.005 * (iteration % 100)) * 255.0 * view.totalWeight);
        const int base = 35 * response.stride + 40;
        const bool greedy = search.safetyMode != ShapeMatch::SearchSafetyV3::Safe;
        const auto scalar = ShapeMatch::scoreBlock16Scalar(response, base, view, threshold, greedy);
        const auto scalar32 = ShapeMatch::scoreBlock32Scalar(response, base, view, threshold);
        const auto scalar64 = ShapeMatch::scoreBlock64Scalar(response, base, view, threshold);
        if (ShapeMatch::cpuSupportsAvx2V3()) {
            const auto avx = ShapeMatch::scoreBlock16AVX2(response, base, view, threshold, greedy);
            const auto avx32 = ShapeMatch::scoreBlock32AVX2(response, base, view, threshold);
            ok &= expect(scalar.rawCosts == avx.rawCosts, "AVX2 integer costs differ from scalar");
            ok &= expect(scalar.validMask == avx.validMask, "AVX2 valid mask differs from scalar");
            ok &= expect(scalar.rejectedStage == avx.rejectedStage, "AVX2 stage rejection differs from scalar");
            ok &= expect(scalar32.rawCosts == avx32.rawCosts, "AVX2 32-lane costs differ from scalar");
            ok &= expect(scalar32.validMask == avx32.validMask, "AVX2 32-lane mask differs from scalar");
        }
        if (ShapeMatch::cpuSupportsAvx512V3()) {
            const auto avx512 = ShapeMatch::scoreBlock64AVX512(response, base, view, threshold);
            ok &= expect(scalar64.rawCosts == avx512.rawCosts,
                         "AVX-512 64-lane costs differ from scalar");
            ok &= expect(scalar64.validMask == avx512.validMask,
                         "AVX-512 64-lane mask differs from scalar");
        }
    }

    return ok;
}

bool parallelAngleViewTest()
{
    ShapeMatch::ShapeModelV3 model;
    for (int i = 0; i < 600; ++i) {
        ShapeMatch::ShapePointV3 point;
        point.x = static_cast<float>((i * 17) % 83 - 41);
        point.y = static_cast<float>((i * 31) % 71 - 35);
        point.quality = static_cast<float>(600 - i);
        point.weight = static_cast<std::uint8_t>(1 + i % 4);
        point.partId = static_cast<std::uint16_t>(i % 9);
        model.points.push_back(point);
    }
    ShapeMatch::ShapeSearchParametersV3 singleParameters;
    singleParameters.enableMultithreading = false;
    ShapeMatch::ShapeSearchParametersV3 parallelParameters = singleParameters;
    parallelParameters.enableMultithreading = true;
    parallelParameters.numThreads = 20;
    const auto single = ShapeMatch::AngleViewBuilderV3().build(
        model, 0.5f, 1024, -3.0f, 6.0f, 0.1f, singleParameters);
    const auto parallel = ShapeMatch::AngleViewBuilderV3().build(
        model, 0.5f, 1024, -3.0f, 6.0f, 0.1f, parallelParameters);
    bool ok = expect(single.size() == parallel.size(),
                     "parallel angle-view count differs from single-thread build");
    for (size_t i = 0; ok && i < single.size(); ++i) {
        const auto& a = single[i];
        const auto& b = parallel[i];
        ok &= expect(a.angleRadians == b.angleRadians && a.totalWeight == b.totalWeight
                         && a.minDx == b.minDx && a.maxDx == b.maxDx
                         && a.minDy == b.minDy && a.maxDy == b.maxDy
                         && a.points.size() == b.points.size() && a.stages.size() == b.stages.size(),
                     "parallel angle-view metadata differs from single-thread build");
        for (size_t p = 0; ok && p < a.points.size(); ++p) {
            const auto& lhs = a.points[p];
            const auto& rhs = b.points[p];
            ok &= expect(lhs.dx == rhs.dx && lhs.dy == rhs.dy
                             && lhs.linearOffset == rhs.linearOffset
                             && lhs.orientationBin == rhs.orientationBin
                             && lhs.weight == rhs.weight && lhs.partId == rhs.partId,
                         "parallel angle-view point differs from single-thread build");
        }
    }
    ShapeMatch::AngleViewBuilderV3 builder;
    builder.precompute(model);
    ok &= expect(model.hasPrecomputedViews(),
                 "V3 model did not cache 36 Level 3 angle views");
    const auto bound = builder.bindPrecomputed(model, 3, 1379);
    ok &= expect(bound.size() == ShapeMatch::kPrecomputedAngleCountV3,
                 "bound V3 Level 3 cache does not contain exactly 36 angles");
    for (const auto& view : bound)
        for (const auto& point : view.points)
            ok &= expect(point.linearOffset == point.dy * 1379 + point.dx,
                         "bound V3 cache has an invalid linear offset");
    for (int level = 0; level < 3; ++level)
        ok &= expect(model.precomputedAngleViews[static_cast<size_t>(level)].empty(),
                     "V3 model unexpectedly cached a non-Level-3 angle view");
    return ok;
}

bool responseMapTest()
{
    cv::Mat image(64, 64, CV_8UC1, cv::Scalar(0));
    cv::line(image, {32, 8}, {32, 55}, cv::Scalar(255), 1);
    ShapeMatch::ResponseMapBuilderV3 builder;
    const auto field = builder.build(image, 10.0f, 30.0f, true);
    bool ok = expect(field.valid() && field.orientationBinCount == 1,
                     "single-plane response-map metadata is invalid");
    const int x = 31, y = 32; // one of the two Canny-localized vertical edges
    ok &= expect(field.binMaps[0].at<std::uint8_t>(y, x) == 0,
                 "Canny edge was not written to the response plane");
    ok &= expect(field.binMaps[0].at<std::uint8_t>(y, 2) == 255,
                 "response spread reached an unrelated location");
    ok &= expect(field.binMaps[0].at<std::uint8_t>(y, x - 1) == 255,
                 "Level 4 response unexpectedly applied spatial spreading");

    cv::Mat mixed(64, 64, CV_8UC1, cv::Scalar(0));
    cv::line(mixed, {8, 20}, {55, 20}, cv::Scalar(255), 1);
    cv::line(mixed, {32, 8}, {32, 55}, cv::Scalar(255), 1);
    const auto single = builder.build(mixed, 10.0f, 30.0f, true);
    ok &= expect(single.valid() && single.orientationBinCount == 1,
                 "direction-agnostic response map did not create exactly one plane");
    ok &= expect(cv::countNonZero(single.binMaps[0] < 255) > 0,
                 "direction-agnostic response plane contains no edge hits");
    return ok;
}

bool exactOrientationBinTest()
{
    ShapeMatch::ResponseMapV3 response;
    response.width = response.height = 32;
    for (int bin = 0; bin < ShapeMatch::kOrientationBinCountV3; ++bin) {
        response.binMaps[static_cast<size_t>(bin)] = cv::Mat(32, 32, CV_8UC1, cv::Scalar(255));
        response.binData[static_cast<size_t>(bin)] = response.binMaps[static_cast<size_t>(bin)].ptr();
    }
    response.stride = static_cast<int>(response.binMaps[0].step[0]);

    ShapeMatch::ShapeModelV3 model;
    ShapeMatch::ShapePointV3 point;
    point.gradientAngleRadians = 0.0f;
    point.quality = 1.0f;
    model.points.push_back(point);
    ShapeMatch::ShapeSearchParametersV3 search;
    search.safetyMode = ShapeMatch::SearchSafetyV3::Safe;
    const auto views = ShapeMatch::AngleViewBuilderV3().build(
        model, 1.0f, response.stride, 0.0f, 0.0f, 1.0f, search);
    const int base = 16 * response.stride + 8;

    response.binMaps[0].at<std::uint8_t>(16, 8) = 0;
    auto score = ShapeMatch::scoreBlock16Scalar(response, base, views[0], 1, false);
    bool ok = expect(score.rawCosts[0] == 0, "same orientation bin did not receive zero distance cost");

    response.binMaps[0].at<std::uint8_t>(16, 8) = 127;
    score = ShapeMatch::scoreBlock16Scalar(response, base, views[0], 1, false);
    ok &= expect(score.rawCosts[0] == 127, "quantized distance cost was not accumulated");
    response.binMaps[0].at<std::uint8_t>(16, 8) = 255;
    score = ShapeMatch::scoreBlock16Scalar(response, base, views[0], 1, false);
    ok &= expect(score.rawCosts[0] == 255, "truncated distance cost was not accumulated");
    return ok;
}

bool strideAndBoundaryTest()
{
    ShapeMatch::ResponseMapV3 response;
    response.width = 129; response.height = 64;
    for (int bin = 0; bin < ShapeMatch::kOrientationBinCountV3; ++bin) {
        cv::Mat storage(80, 180, CV_8UC1, cv::Scalar(0));
        response.binMaps[static_cast<size_t>(bin)] = storage(cv::Rect(13, 7, 129, 64));
        cv::randu(response.binMaps[static_cast<size_t>(bin)], 0, 256);
        response.binData[static_cast<size_t>(bin)] = response.binMaps[static_cast<size_t>(bin)].ptr();
    }
    response.stride = static_cast<int>(response.binMaps[0].step[0]);
    ShapeMatch::ShapeModelV3 model;
    for (int y = -8; y <= 8; y += 4)
        for (int x = -10; x <= 10; x += 4) {
            ShapeMatch::ShapePointV3 point;
            point.x = static_cast<float>(x); point.y = static_cast<float>(y);
            point.quality = 1.0f; point.gradientAngleRadians = 0.0f; point.weight = 1;
            model.points.push_back(point);
        }
    ShapeMatch::ShapeSearchParametersV3 p;
    p.searchRoi = cv::Rect(0, 0, response.width, response.height);
    p.minScore = 0.1f; p.coarseTopK = 20; p.enableAvx2 = true;
    ShapeMatch::PyramidLevelModelV3 lm;
    lm.stride = response.stride;
    lm.angleViews = ShapeMatch::AngleViewBuilderV3().build(model, 1.0f, lm.stride,
                                                           -3.14f, 6.28f, 0.17f, p);
    const auto found = ShapeMatch::CoarseSearchV3().search(response, lm, p);
    for (const auto& c : found) {
        const auto& v = lm.angleViews[static_cast<size_t>(c.angleIndex)];
        if (!expect(c.x + v.minDx >= 0 && c.x + v.maxDx < response.width
                    && c.y + v.minDy >= 0 && c.y + v.maxDy < response.height,
                    "boundary candidate is outside valid view bounds")) return false;
    }
    return true;
}

bool geometryAndClutterTest()
{
    cv::Mat templ(65, 65, CV_8UC1, cv::Scalar(0));
    cv::rectangle(templ, cv::Rect(12, 18, 40, 27), cv::Scalar(255), 2);
    cv::line(templ, {12, 18}, {51, 44}, cv::Scalar(255), 2);
    cv::Mat search(256, 320, CV_8UC1, cv::Scalar(15));
    const cv::Point gt(173, 121);
    cv::Mat affine = cv::getRotationMatrix2D({32, 32}, 10.0, 1.0);
    affine.at<double>(0, 2) += gt.x - 32;
    affine.at<double>(1, 2) += gt.y - 32;
    cv::warpAffine(templ, search, affine, search.size(), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
    cv::rectangle(search, cv::Rect(gt.x - 6, gt.y - 5, 12, 10), cv::Scalar(15), cv::FILLED);
    cv::line(search, {5, 40}, {310, 40}, cv::Scalar(245), 3);
    cv::line(search, {20, 210}, {290, 180}, cv::Scalar(210), 2);
    cv::randn(search(cv::Rect(0, 150, 100, 80)), 45, 30);

    ShapeMatch::ShapeModelParametersV3 mp;
    mp.gradientLow = 15; mp.gradientHigh = 100; mp.pyramidLevels = 4;
    mp.templatePointMinDistance = 2; mp.responseSpreadRadius = 0;
    ShapeMatch::ShapeMatcherV3 matcher;
    const auto model = matcher.createModel(templ, {}, mp);
    ShapeMatch::MatchResultV3 initial;
    initial.pose.x = gt.x + 2.0;
    initial.pose.y = gt.y - 1.5;
    initial.pose.theta = -11.0 * ShapeMatch::kPi / 180.0;
    initial.score = 0.2f;
    ShapeMatch::ShapeSearchParametersV3 refineParameters;
    refineParameters.subpixelSearchRadius = 5.0f;
    refineParameters.subpixelMinCorrespondences = 12;
    const auto refined = ShapeMatch::PoseRefinerV3().refine(search, model, initial, refineParameters);
    const double initialMetric = std::hypot(initial.pose.x - gt.x, initial.pose.y - gt.y)
        + 10.0 * std::abs(ShapeMatch::wrapToPi(initial.pose.theta + 10.0 * ShapeMatch::kPi / 180.0));
    const double refinedMetric = std::hypot(refined.pose.x - gt.x, refined.pose.y - gt.y)
        + 10.0 * std::abs(ShapeMatch::wrapToPi(refined.pose.theta + 10.0 * ShapeMatch::kPi / 180.0));
    bool ok = expect(refined.refined && refined.validCorrespondences >= 12,
                     "continuous pose refiner did not produce enough correspondences");
    ok &= expect(refinedMetric < initialMetric,
                 "continuous pose refinement did not reduce synthetic pose error");
    ShapeMatch::ShapeSearchParametersV3 sp;
    sp.angleStartRadians = -0.35f; sp.angleExtentRadians = 0.7f;
    // Exact orientation bins deliberately provide no adjacent-bin tolerance.
    // Keep the coarse gate below the measured exact-bin response and let the
    // geometric/refinement checks below enforce final pose quality.
    sp.angleStepRadians = static_cast<float>(ShapeMatch::kPi / 18.0); sp.minScore = 0.05f;
    sp.pyramidLevels = 4; sp.maxMatches = 20; sp.coarseTopK = 1000;
    sp.enableSubpixelRefinement = true;
    sp.maxRefineCandidates = 200;
    // This case deliberately occludes the target and adds strong clutter, so it
    // exercises configurable relaxed gates rather than the production defaults.
    sp.refinedMinVisibleRatio = 0.30f;
    sp.refinedMinCorrespondenceRatio = 0.25f;
    sp.refinedMaxRmsResidual = 2.50f;
    sp.safetyMode = ShapeMatch::SearchSafetyV3::Safe;
    ShapeMatch::ShapeMatchStatisticsV3 stats;
    const auto result = matcher.find(search, model, sp, &stats);
    bool near = false;
    int nearCount = 0;
    for (const auto& r : result) {
        ok &= expect(r.refined
                     && r.visibleRatio >= sp.refinedMinVisibleRatio
                     && r.validCorrespondences >= static_cast<int>(std::ceil(
                         model.points.size() * sp.refinedMinCorrespondenceRatio))
                     && r.rmsResidual <= sp.refinedMaxRmsResidual,
                     "V3 final result bypassed the refinement quality gate");
        const double dx = r.pose.x - gt.x, dy = r.pose.y - gt.y;
        // OpenCV's positive image rotation uses the opposite sign from the V3
        // image-coordinate (x right, y down) rotation convention.
        if (std::sqrt(dx * dx + dy * dy) <= 12.0) {
            near = true;
            ++nearCount;
        }
    }
    ok &= expect(near, "synthetic translated/rotated target was not recalled");
    ok &= expect(nearCount == 1, "instance-scale final NMS retained duplicate target poses");
    ShapeMatch::GroundTruthInstance groundTruth;
    groundTruth.id = "synthetic_target";
    groundTruth.pose = ShapeMatch::MatchPose::fromDeg(gt.x, gt.y, -10.0, 1.0);
    auto diagnostic = ShapeMatch::ShapeMatchV3DiagnosticAnalyzer().analyze(
        search, model, sp, stats, result, {groundTruth});
    diagnostic.imageName = "synthetic";
    diagnostic.templateName = "synthetic_template";
    ok &= expect(diagnostic.gtCount == 1 && !diagnostic.groundTruth.empty(),
                 "V3 GT diagnostic did not retain the ground-truth target");
    const std::filesystem::path reportDir =
        std::filesystem::path("data") / "shape_match" / "reports";
    ok &= expect(ShapeMatch::ShapeMatchV3ReportWriter().write(diagnostic, reportDir),
                 "V3 diagnostic report write failed");
    ok &= expect(std::filesystem::exists(reportDir / "latest_shape_match_v3_diagnostic.json"),
                 "V3 diagnostic JSON was not created");
    ok &= expect(std::filesystem::exists(reportDir / "latest_shape_match_v3_gt.csv"),
                 "V3 diagnostic GT CSV was not created");
    return ok;
}
} // namespace

int main()
{
    bool ok = kernelTest();
    ok &= parallelAngleViewTest();
    ok &= responseMapTest();
    ok &= exactOrientationBinTest();
    ok &= strideAndBoundaryTest();
    ok &= geometryAndClutterTest();
    std::cout << (ok ? "[ShapeMatchV3] all tests passed\n" : "[ShapeMatchV3] tests failed\n");
    return ok ? 0 : 1;
}
