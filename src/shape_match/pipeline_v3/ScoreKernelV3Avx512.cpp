#include "shape_match/pipeline_v3/ScoreKernelV3.h"

#include <immintrin.h>

namespace ShapeMatch {

ScoreBlock64V3 scoreBlock64AVX512(const ResponseMapV3& responseMap, std::int32_t centerBase,
                                  const AngleViewV3& view, std::uint32_t finalRawThreshold)
{
    ScoreBlock64V3 out;
    __m512i sum0 = _mm512_setzero_si512();
    __m512i sum1 = _mm512_setzero_si512();
    __m512i sum2 = _mm512_setzero_si512();
    __m512i sum3 = _mm512_setzero_si512();

    for (const RotatedPointV3& point : view.points) {
        const auto* src = responseMap.binData[point.orientationBin]
                        + centerBase + point.linearOffset;
        const __m512i packed64 = _mm512_loadu_si512(src);
        __m512i values0 = _mm512_cvtepu8_epi32(
            _mm512_extracti32x4_epi32(packed64, 0));
        __m512i values1 = _mm512_cvtepu8_epi32(
            _mm512_extracti32x4_epi32(packed64, 1));
        __m512i values2 = _mm512_cvtepu8_epi32(
            _mm512_extracti32x4_epi32(packed64, 2));
        __m512i values3 = _mm512_cvtepu8_epi32(
            _mm512_extracti32x4_epi32(packed64, 3));
        if (point.weight != 1) {
            const __m512i weight = _mm512_set1_epi32(point.weight);
            values0 = _mm512_mullo_epi32(values0, weight);
            values1 = _mm512_mullo_epi32(values1, weight);
            values2 = _mm512_mullo_epi32(values2, weight);
            values3 = _mm512_mullo_epi32(values3, weight);
        }
        sum0 = _mm512_add_epi32(sum0, values0);
        sum1 = _mm512_add_epi32(sum1, values1);
        sum2 = _mm512_add_epi32(sum2, values2);
        sum3 = _mm512_add_epi32(sum3, values3);
    }

    _mm512_store_si512(out.rawCosts.data(), sum0);
    _mm512_store_si512(out.rawCosts.data() + 16, sum1);
    _mm512_store_si512(out.rawCosts.data() + 32, sum2);
    _mm512_store_si512(out.rawCosts.data() + 48, sum3);
    const __m512i threshold = _mm512_set1_epi32(static_cast<int>(finalRawThreshold));
    out.validMask = static_cast<std::uint64_t>(_mm512_cmple_epu32_mask(sum0, threshold))
        | (static_cast<std::uint64_t>(_mm512_cmple_epu32_mask(sum1, threshold)) << 16)
        | (static_cast<std::uint64_t>(_mm512_cmple_epu32_mask(sum2, threshold)) << 32)
        | (static_cast<std::uint64_t>(_mm512_cmple_epu32_mask(sum3, threshold)) << 48);
    return out;
}

} // namespace ShapeMatch
