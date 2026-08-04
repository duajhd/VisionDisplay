#pragma once

#include "shape_match/coarse/OrientationResponseConfig.h"
#include "shape_match/coarse/OrientationResponseMap.h"
#include "shape_match/coarse/ResponseTemplate.h"

#include <opencv2/core.hpp>

namespace ShapeMatch {

class ResponseMapEvaluator
{
public:
    cv::Mat evaluateTheta(const OrientationResponseMap& responseMap,
                          const ResponseTemplate& responseTemplate,
                          int thetaBin,
                          double thetaRad,
                          const OrientationResponseConfig& config) const;
};

} // namespace ShapeMatch
