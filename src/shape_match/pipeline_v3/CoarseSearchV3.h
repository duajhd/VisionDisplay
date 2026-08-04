#pragma once

#include "shape_match/pipeline_v3/ShapeMatchV3Types.h"

namespace ShapeMatch {

class WorkerPoolV3;

class CandidateCollectorV3
{
public:
    static void keepTopK(std::vector<MatchCandidateV3>& candidates, int capacity);
};

class PoseNmsV3
{
public:
    std::vector<MatchCandidateV3> apply(const std::vector<MatchCandidateV3>& sortedCandidates,
                                       const std::vector<AngleViewV3>& views,
                                       int maxCount, float positionDistance,
                                       float angleDistance) const;
};

class CoarseSearchV3
{
public:
    std::vector<MatchCandidateV3> search(const ResponseMapV3& response,
                                        const PyramidLevelModelV3& model,
                                        const ShapeSearchParametersV3& parameters,
                                        ShapeMatchStatisticsV3* statistics = nullptr,
                                        WorkerPoolV3* workerPool = nullptr) const;
};

} // namespace ShapeMatch
