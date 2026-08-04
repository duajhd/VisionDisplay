#pragma once

#include "shape_match/pipeline_v2/IShapeCandidateGenerator.h"

namespace ShapeMatch {

class OldVotingCandidateGeneratorAdapter : public IShapeCandidateGenerator
{
public:
    std::vector<MatchPose> generate(const ShapeMatchPipelineV2Context& context,
                                    const ShapeMatchPipelineV2Config& config,
                                    ShapeMatchPipelineV2Result* result) override;

    const char* name() const override;
};

} // namespace ShapeMatch
