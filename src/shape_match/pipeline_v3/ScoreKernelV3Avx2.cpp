#include "shape_match/pipeline_v3/ScoreKernelV3.h"

#include <immintrin.h>
#include <algorithm>

namespace ShapeMatch {

ScoreBlock16V3 scoreBlock16AVX2(const ResponseMapV3& responseMap, std::int32_t centerBase,
                                const AngleViewV3& view, std::uint32_t finalRawThreshold,
                                bool)
{
    ScoreBlock16V3 out;
    __m256i lowSum = _mm256_setzero_si256();
    __m256i highSum = _mm256_setzero_si256();
    alignas(32) std::uint32_t sums[16] {};

    // A lane is one horizontal candidate center. For every fixed template point,
    // the 16 queried bytes are contiguous and therefore require no gather.
    for (const RotatedPointV3& point : view.points) {
            const auto* src = responseMap.binData[point.orientationBin] + centerBase + point.linearOffset;
            const __m128i bytes = _mm_loadu_si128(reinterpret_cast<const __m128i*>(src));
            __m256i low = _mm256_cvtepu8_epi32(bytes);
            __m256i high = _mm256_cvtepu8_epi32(_mm_srli_si128(bytes, 8));
            if (point.weight != 1) {
                const __m256i weight = _mm256_set1_epi32(point.weight);
                low = _mm256_mullo_epi32(low, weight);
                high = _mm256_mullo_epi32(high, weight);
            }
            lowSum = _mm256_add_epi32(lowSum, low);
            highSum = _mm256_add_epi32(highSum, high);
    }
    _mm256_store_si256(reinterpret_cast<__m256i*>(sums), lowSum);
    _mm256_store_si256(reinterpret_cast<__m256i*>(sums + 8), highSum);
    for (int i = 0; i < 16; ++i) out.rawCosts[static_cast<size_t>(i)] = sums[i];
    for (int lane = 0; lane < 16; ++lane)
        if (sums[lane] <= finalRawThreshold)
            out.validMask = static_cast<std::uint16_t>(out.validMask | (1u << lane));
    return out;
}

ScoreBlock32V3 scoreBlock32AVX2(const ResponseMapV3& responseMap, std::int32_t centerBase,
                                const AngleViewV3& view, std::uint32_t finalRawThreshold)
{
    ScoreBlock32V3 out;
    __m256i sum0 = _mm256_setzero_si256();
    __m256i sum1 = _mm256_setzero_si256();
    __m256i sum2 = _mm256_setzero_si256();
    __m256i sum3 = _mm256_setzero_si256();

    // One unaligned 256-bit load fetches the response costs for 32 adjacent
    // horizontal candidate centers. Four uint32 vectors avoid overflow.
    for (const RotatedPointV3& point : view.points) {
        const auto* src = responseMap.binData[point.orientationBin]
                        + centerBase + point.linearOffset;
        const __m256i packed32 = _mm256_loadu_si256(
            reinterpret_cast<const __m256i*>(src));
        const __m128i packedLow = _mm256_castsi256_si128(packed32);
        const __m128i packedHigh = _mm256_extracti128_si256(packed32, 1);
        __m256i values0 = _mm256_cvtepu8_epi32(packedLow);
        __m256i values1 = _mm256_cvtepu8_epi32(_mm_srli_si128(packedLow, 8));
        __m256i values2 = _mm256_cvtepu8_epi32(packedHigh);
        __m256i values3 = _mm256_cvtepu8_epi32(_mm_srli_si128(packedHigh, 8));
        if (point.weight != 1) {
            const __m256i weight = _mm256_set1_epi32(point.weight);
            values0 = _mm256_mullo_epi32(values0, weight);
            values1 = _mm256_mullo_epi32(values1, weight);
            values2 = _mm256_mullo_epi32(values2, weight);
            values3 = _mm256_mullo_epi32(values3, weight);
        }
        sum0 = _mm256_add_epi32(sum0, values0);
        sum1 = _mm256_add_epi32(sum1, values1);
        sum2 = _mm256_add_epi32(sum2, values2);
        sum3 = _mm256_add_epi32(sum3, values3);
    }

    _mm256_store_si256(reinterpret_cast<__m256i*>(out.rawCosts.data()), sum0);
    _mm256_store_si256(reinterpret_cast<__m256i*>(out.rawCosts.data() + 8), sum1);
    _mm256_store_si256(reinterpret_cast<__m256i*>(out.rawCosts.data() + 16), sum2);
    _mm256_store_si256(reinterpret_cast<__m256i*>(out.rawCosts.data() + 24), sum3);
    for (int lane = 0; lane < 32; ++lane)
        if (out.rawCosts[static_cast<size_t>(lane)] <= finalRawThreshold)
            out.validMask |= (std::uint32_t{1} << lane);
    return out;
}

} // namespace ShapeMatch
