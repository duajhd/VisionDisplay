#pragma once

#include "shape_match/pipeline_v3/ShapeMatchV3Types.h"

namespace ShapeMatch {

ScoreBlock16V3 scoreBlock16Scalar(const ResponseMapV3& responseMap, std::int32_t centerBase,
                                  const AngleViewV3& view, std::uint32_t finalRawThreshold,
                                  bool enableGreedy);
ScoreBlock16V3 scoreBlock16AVX2(const ResponseMapV3& responseMap, std::int32_t centerBase,
                                const AngleViewV3& view, std::uint32_t finalRawThreshold,
                                bool enableGreedy);
ScoreBlock32V3 scoreBlock32Scalar(const ResponseMapV3& responseMap, std::int32_t centerBase,
                                  const AngleViewV3& view, std::uint32_t finalRawThreshold);
ScoreBlock32V3 scoreBlock32AVX2(const ResponseMapV3& responseMap, std::int32_t centerBase,
                                const AngleViewV3& view, std::uint32_t finalRawThreshold);
ScoreBlock64V3 scoreBlock64Scalar(const ResponseMapV3& responseMap, std::int32_t centerBase,
                                  const AngleViewV3& view, std::uint32_t finalRawThreshold);
ScoreBlock64V3 scoreBlock64AVX512(const ResponseMapV3& responseMap, std::int32_t centerBase,
                                  const AngleViewV3& view, std::uint32_t finalRawThreshold);
bool cpuSupportsAvx2V3();
bool cpuSupportsAvx512V3();

} // namespace ShapeMatch
