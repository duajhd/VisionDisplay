#pragma once

#include "shape_match/core/ShapeMatchTypes.h"
#include "shape_match/evaluation/ShapeMatchEvalConfig.h"

#include <vector>

namespace ShapeMatch {

struct FinalCandidateNmsResult
{
    std::vector<MatchPose> poses;
    int inputCount = 0;
    int afterNmsCount = 0;
};

class FinalCandidateNms
{
public:
    explicit FinalCandidateNms(ShapeMatchEvalConfig config = {});

    FinalCandidateNmsResult apply(const std::vector<MatchPose>& candidates) const;

private:
    bool isDuplicate(const MatchPose& a, const MatchPose& b) const;

    ShapeMatchEvalConfig m_config;
};

} // namespace ShapeMatch
