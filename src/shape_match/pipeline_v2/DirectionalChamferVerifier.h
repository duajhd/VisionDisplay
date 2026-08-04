#pragma once

#include "shape_match/pipeline_v2/DirectionalChamferScore.h"
#include "shape_match/pipeline_v2/DirectionalDistanceField.h"
#include "shape_match/pipeline_v2/ResponsePeakVerifier.h"
#include "shape_match/pipeline_v2/SegmentTemplate.h"

namespace ShapeMatch {

class DirectionalChamferVerifier
{
public:
    explicit DirectionalChamferVerifier(DirectionalChamferConfig config = {});

    DirectionalChamferScore scoreCandidate(const MatchPose& pose,
                                           const SegmentTemplate& templ,
                                           const DirectionalDistanceField& field,
                                           double pruningFloor = -1.0) const;

    std::vector<VerifiedResponsePeak> verifyPeaks(const std::vector<VerifiedResponsePeak>& input,
                                                  const SegmentTemplate& templ,
                                                  const DirectionalDistanceField& field,
                                                  std::vector<DirectionalChamferScore>* scores = nullptr,
                                                  int* prunedCount = nullptr) const;

private:
    DirectionalChamferConfig m_config;
};

} // namespace ShapeMatch
