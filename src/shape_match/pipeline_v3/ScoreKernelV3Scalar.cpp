#include "shape_match/pipeline_v3/ScoreKernelV3.h"

#include <opencv2/core/utility.hpp>

#include <algorithm>
#include <limits>

namespace ShapeMatch {
ScoreBlock16V3 scoreBlock16Scalar(const ResponseMapV3& responseMap, std::int32_t centerBase,
                                  const AngleViewV3& view, std::uint32_t finalRawThreshold,
                                  bool)
{
    ScoreBlock16V3 out;
    std::array<std::uint32_t, 16> sums {};
    for (const RotatedPointV3& point : view.points) {
        const std::uint8_t* src = responseMap.binData[point.orientationBin]
                                + centerBase + point.linearOffset;
        for (int lane = 0; lane < 16; ++lane)
            sums[static_cast<size_t>(lane)] += static_cast<std::uint32_t>(src[lane]) * point.weight;
    }
    out.rawCosts = sums;
    for (int lane = 0; lane < 16; ++lane)
        if (sums[static_cast<size_t>(lane)] <= finalRawThreshold)
            out.validMask = static_cast<std::uint16_t>(out.validMask | (1u << lane));
    return out;
}

ScoreBlock32V3 scoreBlock32Scalar(const ResponseMapV3& responseMap, std::int32_t centerBase,
                                  const AngleViewV3& view, std::uint32_t finalRawThreshold)
{
    ScoreBlock32V3 out;
    for (const RotatedPointV3& point : view.points) {
        const std::uint8_t* src = responseMap.binData[point.orientationBin]
                                + centerBase + point.linearOffset;
        for (int lane = 0; lane < 32; ++lane)
            out.rawCosts[static_cast<size_t>(lane)] += static_cast<std::uint32_t>(src[lane]) * point.weight;
    }
    for (int lane = 0; lane < 32; ++lane)
        if (out.rawCosts[static_cast<size_t>(lane)] <= finalRawThreshold)
            out.validMask |= (std::uint32_t{1} << lane);
    return out;
}

ScoreBlock64V3 scoreBlock64Scalar(const ResponseMapV3& responseMap, std::int32_t centerBase,
                                  const AngleViewV3& view, std::uint32_t finalRawThreshold)
{
    ScoreBlock64V3 out;
    for (const RotatedPointV3& point : view.points) {
        const std::uint8_t* src = responseMap.binData[point.orientationBin]
                                + centerBase + point.linearOffset;
        for (int lane = 0; lane < 64; ++lane)
            out.rawCosts[static_cast<size_t>(lane)] += static_cast<std::uint32_t>(src[lane]) * point.weight;
    }
    for (int lane = 0; lane < 64; ++lane)
        if (out.rawCosts[static_cast<size_t>(lane)] <= finalRawThreshold)
            out.validMask |= (std::uint64_t{1} << lane);
    return out;
}

bool cpuSupportsAvx2V3()
{
    return cv::checkHardwareSupport(CV_CPU_AVX2);
}

bool cpuSupportsAvx512V3()
{
    return cv::checkHardwareSupport(CV_CPU_AVX_512F)
        && cv::checkHardwareSupport(CV_CPU_AVX_512DQ);
}

} // namespace ShapeMatch
