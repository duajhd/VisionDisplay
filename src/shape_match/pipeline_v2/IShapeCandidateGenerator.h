#pragma once

#include "shape_match/core/ShapeMatchTypes.h"
#include "shape_match/pipeline_v2/ShapeMatchPipelineV2Config.h"
#include "shape_match/pipeline_v2/ShapeMatchPipelineV2Context.h"
#include "shape_match/pipeline_v2/ShapeMatchPipelineV2Result.h"

#include <vector>

namespace ShapeMatch {

class IShapeCandidateGenerator
{
public:
    virtual ~IShapeCandidateGenerator() = default;

    virtual std::vector<MatchPose> generate(const ShapeMatchPipelineV2Context& context,
                                            const ShapeMatchPipelineV2Config& config,
                                            ShapeMatchPipelineV2Result* result) = 0;

    virtual const char* name() const = 0;
};

} // namespace ShapeMatch
