#pragma once

#include "shape_match/core/ShapeMatchTypes.h"

#include <filesystem>
#include <string>
#include <vector>

namespace ShapeMatch {

struct VisionProTrainingInfo
{
    bool hasTrainingRoi = false;
    double roiX = 0.0;
    double roiY = 0.0;
    double roiWidth = 0.0;
    double roiHeight = 0.0;

    bool hasOrigin = false;
    double originX = 0.0;
    double originY = 0.0;
    std::string originKind = "template_origin";
};

struct VisionProGroundTruthReadResult
{
    std::vector<GroundTruthInstance> instances;
    std::string imageName;
    std::string templateName;
    std::string source = "visionpro_cogpmalign";
    std::string poseOrigin = "template_origin";
    VisionProTrainingInfo training;
    std::vector<std::string> warnings;
    std::string error;

    bool ok() const
    {
        return error.empty();
    }
};

class VisionProGroundTruthReader
{
public:
    VisionProGroundTruthReadResult read(const std::filesystem::path& path) const;

    static const char* schemaName();
    static std::string exampleJson();
};

} // namespace ShapeMatch
