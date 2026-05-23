#pragma once

#include "shape_match/coarse/VotingConfig.h"
#include "shape_match/coarse/VotingDebugReport.h"
#include "shape_match/core/EdgeImageData.h"

#include <vector>

namespace ShapeMatch {

class OrientationVotingCandidateGenerator
{
public:
    explicit OrientationVotingCandidateGenerator(VotingConfig config = {});

    std::vector<MatchPose> generate(const ShapeTemplateModel& model,
                                    const EdgeImageData& edgeData,
                                    VotingDebugReport* debugReport = nullptr) const;

private:
    void writeDebugReport(const VotingDebugReport& report) const;

    VotingConfig m_config;
};

} // namespace ShapeMatch
