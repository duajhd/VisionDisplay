#pragma once

#include "shape_match/coarse/OrientationResponseConfig.h"
#include "shape_match/coarse/ResponseDebugReport.h"
#include "shape_match/core/EdgeImageData.h"
#include "shape_match/core/ShapeTemplateModel.h"

#include <vector>

namespace ShapeMatch {

class ResponseCandidateGenerator
{
public:
    explicit ResponseCandidateGenerator(OrientationResponseConfig config = {});

    std::vector<MatchPose> generate(const ShapeTemplateModel& templateModel,
                                    const EdgeImageData& edgeData,
                                    int pyramidLevel,
                                    ResponseDebugReport* debugReport = nullptr) const;

    std::vector<ResponsePeak> generatePeaks(const ShapeTemplateModel& templateModel,
                                            const EdgeImageData& edgeData,
                                            int pyramidLevel,
                                            ResponseDebugReport* debugReport = nullptr) const;

private:
    OrientationResponseConfig m_config;
};

} // namespace ShapeMatch
