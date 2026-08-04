#include "shape_match/pipeline_v3/AngleViewBuilderV3.h"
#include "shape_match/pipeline_v3/CoarseSearchV3.h"
#include "shape_match/pipeline_v3/ResponseMapBuilderV3.h"
#include "shape_match/pipeline_v3/ScoreKernelV3.h"

#include <opencv2/core.hpp>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>

int main()
{
    std::cout << std::unitbuf;
    using Clock = std::chrono::steady_clock;
    ShapeMatch::ResponseMapV3 response;
    response.width = response.height = 512;
    for (int bin = 0; bin < ShapeMatch::kOrientationBinCountV3; ++bin) {
        response.binMaps[static_cast<size_t>(bin)].create(512, 512, CV_8UC1);
        cv::randu(response.binMaps[static_cast<size_t>(bin)], 0, 256);
        response.binData[static_cast<size_t>(bin)] = response.binMaps[static_cast<size_t>(bin)].ptr();
    }
    response.stride = static_cast<int>(response.binMaps[0].step[0]);
    std::cout << "points,angles,single_candidate_x32_ms,scalar32_ms,avx2_32_ms,"
                 "avx2_vs_single,avx2_vs_scalar32,candidates_per_second_avx2\n";
    for (int pointCount : {48, 128, 256, 512, 1024}) {
        ShapeMatch::ShapeModelV3 model;
        const int side = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(pointCount))));
        for (int i = 0; i < pointCount; ++i) {
            ShapeMatch::ShapePointV3 point;
            point.x = static_cast<float>((i % side) * 2 - side);
            point.y = static_cast<float>((i / side) * 2 - side);
            point.quality = static_cast<float>(pointCount - i);
            point.gradientAngleRadians = static_cast<float>((i % ShapeMatch::kOrientationBinCountV3)
                * 2.0 * ShapeMatch::kPi / ShapeMatch::kOrientationBinCountV3);
            point.weight = static_cast<std::uint8_t>(1 + i % 3);
            model.points.push_back(point);
        }
        for (int angles : {36, 72, 180, 360}) {
            ShapeMatch::ShapeSearchParametersV3 p;
            p.safetyMode = ShapeMatch::SearchSafetyV3::Safe;
            const auto views = ShapeMatch::AngleViewBuilderV3().build(
                model, 1.0f, response.stride, 0.0f,
                static_cast<float>(2 * ShapeMatch::kPi),
                static_cast<float>(2 * ShapeMatch::kPi / angles), p);
            constexpr int blocks = 2000;
            auto run = [&](bool avx) {
                volatile std::uint32_t sink = 0;
                const auto t0 = Clock::now();
                for (int i = 0; i < blocks; ++i) {
                    const auto& v = views[static_cast<size_t>(i % views.size())];
                    const int x = 64 + (i % 360), y = 64 + ((i / 360) % 360);
                    auto b = avx ? ShapeMatch::scoreBlock32AVX2(response, y * response.stride + x, v, 0)
                                 : ShapeMatch::scoreBlock32Scalar(response, y * response.stride + x, v, 0);
                    sink = sink + b.rawCosts[0];
                }
                (void)sink;
                return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
            };
            auto runSingle = [&] {
                volatile std::uint32_t sink = 0;
                const auto t0 = Clock::now();
                for (int i = 0; i < blocks; ++i) {
                    const auto& v = views[static_cast<size_t>(i % views.size())];
                    const int x = 64 + (i % 360), y = 64 + ((i / 360) % 360);
                    for (int lane = 0; lane < 32; ++lane) {
                        std::uint32_t sum = 0;
                        const int base = y * response.stride + x + lane;
                        for (const auto& point : v.points)
                            sum += static_cast<std::uint32_t>(response.binData[point.orientationBin][base + point.linearOffset]) * point.weight;
                        sink = sink + sum;
                    }
                }
                (void)sink;
                return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
            };
            const double single = runSingle();
            const double scalar = run(false);
            const double avx = ShapeMatch::cpuSupportsAvx2V3() ? run(true) : scalar;
            const double cps = blocks * 32.0 / (avx * 0.001);
            std::cout << pointCount << ',' << angles << ',' << std::fixed << std::setprecision(3)
                      << single << ',' << scalar << ',' << avx << ',' << single / avx
                      << ',' << scalar / avx << ',' << cps << '\n';
        }
    }

    std::cout << "\nresponse_image,planes,time_ms\n";
    ShapeMatch::ResponseMapBuilderV3 responseBuilder;
    for (int size : {1024, 2048}) {
        cv::Mat image(size, size, CV_8UC1);
        cv::randu(image, 0, 256);
        const auto t0 = Clock::now();
        auto built = responseBuilder.build(image, 20.0f, 100.0f, false);
        const double elapsed = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        std::cout << size << 'x' << size << ",1," << elapsed << '\n';
        if (!built.valid() || built.orientationBinCount != 1) return 2;
    }

    std::cout << "\ncoarse_size,safety,threads,time_ms,candidates_per_second,"
                 "stage0_reject,stage1_reject,stage2_reject,fully_evaluated,avx2_blocks\n";
    ShapeMatch::ShapeModelV3 denseModel;
    for (int i = 0; i < 128; ++i) {
        ShapeMatch::ShapePointV3 point;
        point.x = static_cast<float>((i % 16) * 2 - 15);
        point.y = static_cast<float>((i / 16) * 3 - 11);
        point.quality = static_cast<float>(128 - i);
        point.gradientAngleRadians = static_cast<float>((i % ShapeMatch::kOrientationBinCountV3)
            * 2.0 * ShapeMatch::kPi / ShapeMatch::kOrientationBinCountV3);
        point.weight = static_cast<std::uint8_t>(1 + i % 3);
        denseModel.points.push_back(point);
    }
    for (int size : {128, 256, 512}) {
        ShapeMatch::ResponseMapV3 rm;
        rm.width = size; rm.height = size;
        for (int bin = 0; bin < ShapeMatch::kOrientationBinCountV3; ++bin) {
            rm.binMaps[static_cast<size_t>(bin)].create(size, size + 17, CV_8UC1);
            cv::randu(rm.binMaps[static_cast<size_t>(bin)], 0, 256);
            rm.binData[static_cast<size_t>(bin)] = rm.binMaps[static_cast<size_t>(bin)].ptr();
        }
        rm.stride = static_cast<int>(rm.binMaps[0].step[0]);
        for (int safetyIndex = 0; safetyIndex < 3; ++safetyIndex) {
            for (int threaded : {0, 1}) {
                ShapeMatch::ShapeSearchParametersV3 p;
                p.safetyMode = static_cast<ShapeMatch::SearchSafetyV3>(safetyIndex);
                p.enableMultithreading = threaded != 0;
                p.numThreads = threaded ? 0 : 1;
                p.minScore = 0.5f; p.coarseTopK = 100;
                p.angleStartRadians = 0.0f;
                p.angleExtentRadians = static_cast<float>(2 * ShapeMatch::kPi);
                p.angleStepRadians = static_cast<float>(2 * ShapeMatch::kPi / 72.0);
                ShapeMatch::PyramidLevelModelV3 lm;
                lm.stride = rm.stride;
                lm.angleViews = ShapeMatch::AngleViewBuilderV3().build(
                    denseModel, 1.0f, lm.stride, p.angleStartRadians,
                    p.angleExtentRadians, p.angleStepRadians, p);
                ShapeMatch::ShapeMatchStatisticsV3 stats;
                const auto t0 = Clock::now();
                auto candidates = ShapeMatch::CoarseSearchV3().search(rm, lm, p, &stats);
                const double elapsed = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
                const double cps = stats.evaluatedCandidates / (elapsed * 0.001);
                std::cout << size << ',' << safetyIndex << ',' << (threaded ? "multi" : "single")
                          << ',' << elapsed << ',' << cps << ',' << stats.rejectedAtStage0
                          << ',' << stats.rejectedAtStage1 << ',' << stats.rejectedAtStage2
                          << ',' << stats.fullyEvaluated << ',' << stats.avx2Blocks << '\n';
                (void)candidates;
            }
        }
    }
    std::cout << "\nbottleneck_observation,single-plane response construction scales with image pixels; "
                 "coarse scoring is dominated by plane memory traffic and point count; "
                 "task scheduling dominates small coarse levels; NMS is negligible at bounded Top-K.\n";
    return 0;
}
