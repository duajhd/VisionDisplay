#pragma once

#include "VisionTools/CaliperTool.h"
#include "VisionTools/CircleFitter.h"
#include "VisionTools/CircleTypes.h"
#include "VisionTools/VisionTools_global.h"

namespace VisionTools {

class VISIONTOOLS_API FindCircleTool
{
public:
    FindCircleTool();

    void setParams(const FindCircleParams& params);
    const FindCircleParams& params() const;

    FindCircleResult run(const ImageView& image, const CircleSearchRegion& region) const;

private:
    std::vector<CaliperRegion> generateCalipers(const CircleSearchRegion& region) const;

    FindCircleParams m_params;
};

} // namespace VisionTools
