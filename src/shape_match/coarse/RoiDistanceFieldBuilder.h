#pragma once

#include "shape_match/coarse/CoarseMatchConfig.h"
#include "shape_match/coarse/RoiDistanceField.h"

namespace ShapeMatch {

class RoiDistanceFieldBuilder
{
public:
    RoiDistanceFieldSet buildForRegions(const EdgeImageData& fullLevelEdgeData,
                                        const std::vector<cv::Rect>& rois,
                                        int level,
                                        double levelScale,
                                        const CoarseMatchConfig::RoiDistanceFieldConfig& config) const;

private:
    std::vector<cv::Rect> normalizeRois(const std::vector<cv::Rect>& rois,
                                        const cv::Size& imageSize,
                                        int level,
                                        const CoarseMatchConfig::RoiDistanceFieldConfig& config) const;
};

} // namespace ShapeMatch
