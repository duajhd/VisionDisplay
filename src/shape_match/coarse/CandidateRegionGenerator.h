#pragma once

#include "shape_match/coarse/CoarseCandidate.h"
#include "shape_match/coarse/CoarseMatchConfig.h"
#include "shape_match/core/ShapeTemplateModel.h"

#include <opencv2/core.hpp>

#include <vector>

namespace ShapeMatch {

struct CandidateRegion
{
    cv::Rect roi;
    double score = 0.0;
    int sourceCandidateCount = 0;
};

class CandidateRegionGenerator
{
public:
    explicit CandidateRegionGenerator(CoarseMatchConfig config = {});

    std::vector<CandidateRegion> generateLevelRegions(const std::vector<CoarseCandidate>& seedCandidates,
                                                      const ShapeTemplateModel& templateLevel,
                                                      cv::Size imageSize,
                                                      int level) const;

private:
    CoarseMatchConfig m_config;
};

} // namespace ShapeMatch
