#pragma once

#include "VisionTools/CaliperTool.h"
#include "VisionTools/LineFitter.h"
#include "VisionTools/LineTypes.h"
#include "VisionTools/VisionTools_global.h"

namespace VisionTools {

class VISIONTOOLS_API FindLineTool
{
public:
    FindLineTool();

    void setParams(const FindLineParams& params);
    const FindLineParams& params() const;

    FindLineResult run(const ImageView& image, const LineSearchRegion& region) const;

private:
    std::vector<CaliperRegion> generateCalipers(const LineSearchRegion& region) const;

    FindLineParams m_params;
};

} // namespace VisionTools
