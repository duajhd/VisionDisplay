#pragma once

#include "shape_match/pipeline_v2/ShapeMatchPipelineV2Config.h"
#include "shape_match/pipeline_v2/ShapeMatchPipelineV2Context.h"
#include "shape_match/pipeline_v2/ShapeMatchPipelineV2Result.h"

namespace ShapeMatch {

class ShapeMatchPipelineV2
{
public:
    explicit ShapeMatchPipelineV2(ShapeMatchPipelineV2Config config = {});

    ShapeMatchPipelineV2Result run(const ShapeMatchPipelineV2Context& context);

private:
    ShapeMatchPipelineV2Config config_;
};

} // namespace ShapeMatch
