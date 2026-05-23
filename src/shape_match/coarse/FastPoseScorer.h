#pragma once

#include "shape_match/coarse/CoarseCandidate.h"
#include "shape_match/coarse/CoarseMatchConfig.h"
#include "shape_match/coarse/RoiDistanceField.h"
#include "shape_match/coarse/RotatedTemplateCache.h"
#include "shape_match/coarse/TemplatePointSoA.h"
#include "shape_match/core/EdgeImageData.h"

#include <vector>

namespace ShapeMatch {

struct FastScoreContext
{
    float currentTopKMinScore = -1.0f;
    bool hasTopKThreshold = false;
};

class FastPoseScorer
{
public:
    explicit FastPoseScorer(CoarseMatchConfig config = {});

    CoarseCandidate scorePose(const ShapeTemplateModel& templateLevel,
                              const EdgeImageData& imageLevel,
                              const MatchPose& pose,
                              int pyramidLevel,
                              double pruningScoreFloor = -1.0) const;

    CoarseCandidate scorePose(const ShapeTemplateModel& templateLevel,
                              const EdgeImageData& imageLevel,
                              const MatchPose& pose,
                              int pyramidLevel,
                              const FastScoreContext& context) const;

    CoarseCandidate scorePoseFast(const TemplatePointSoA& points,
                                  const RotatedTemplateCache& rotationCache,
                                  const EdgeImageData& imageLevel,
                                  const MatchPose& pose,
                                  int pyramidLevel,
                                  const FastScoreContext& context) const;

    CoarseCandidate scorePoseFast(const TemplatePointSoA& points,
                                  const RotatedTemplateCache& rotationCache,
                                  const EdgeQueryContext& queryContext,
                                  const MatchPose& pose,
                                  int pyramidLevel,
                                  const FastScoreContext& context) const;

    CoarseCandidate scorePoseWithContext(const ShapeTemplateModel& templateLevel,
                                         const EdgeQueryContext& queryContext,
                                         const MatchPose& pose,
                                         int pyramidLevel,
                                         const FastScoreContext& context) const;

private:
    CoarseCandidate scorePoseDistanceField(const ShapeTemplateModel& templateLevel,
                                           const EdgeImageData& imageLevel,
                                           const MatchPose& pose,
                                           int pyramidLevel,
                                           const FastScoreContext& context) const;

    CoarseCandidate scorePoseLocalSearch(const ShapeTemplateModel& templateLevel,
                                         const EdgeImageData& imageLevel,
                                         const MatchPose& pose,
                                         int pyramidLevel,
                                         const FastScoreContext& context) const;

    std::vector<int> orderedPointIndices(const ShapeTemplateModel& templateLevel) const;
    double distanceScore(double distance) const;
    double orientationScore(double angleDiffDeg) const;
    double polarityScore(bool polarityOk, EdgePolarity polarity) const;

    CoarseMatchConfig m_config;
};

} // namespace ShapeMatch
