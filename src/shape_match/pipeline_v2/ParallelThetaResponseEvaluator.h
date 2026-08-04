#pragma once

#include "shape_match/coarse/OrientationResponseMap.h"
#include "shape_match/coarse/ResponseDebugReport.h"
#include "shape_match/coarse/ResponseTemplate.h"
#include "shape_match/pipeline_v2/RuntimeBufferPool.h"
#include "shape_match/pipeline_v2/ShapeMatchPipelineV2Config.h"

namespace ShapeMatch {

class ParallelThetaResponseEvaluator
{
public:
    explicit ParallelThetaResponseEvaluator(const PipelineV2ProductionConfig& productionConfig);

    std::vector<ResponsePeak> evaluateAllTheta(const OrientationResponseMap& responseMap,
                                               const ResponseTemplate& responseTemplate,
                                               const OrientationResponseConfig& responseConfig,
                                               ResponseDebugReport* debugReport,
                                               RuntimeBufferPool* bufferPool) const;

private:
    PipelineV2ProductionConfig m_config;
};

} // namespace ShapeMatch
