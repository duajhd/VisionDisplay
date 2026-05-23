#pragma once

#include "VisionTools/CaliperTypes.h"
#include "VisionTools/ImageView.h"
#include "VisionTools/VisionTools_global.h"

#include <vector>

namespace VisionTools {

class VISIONTOOLS_API CaliperTool
{
public:
    CaliperTool();

    void setParams(const CaliperParams& params);
    const CaliperParams& params() const;

    CaliperResult run(const ImageView& image, const CaliperRegion& region) const;

private:
    std::vector<double> sampleProfile(const ImageView& image,
                                      const CaliperRegion& region,
                                      bool* hadInvalidSamples) const;

    std::vector<double> smoothProfile(const std::vector<double>& profile) const;

    std::vector<double> computeGradient(const std::vector<double>& profile) const;

    std::vector<EdgePoint> detectCandidates(const std::vector<double>& gradient,
                                            const CaliperRegion& region) const;

    std::vector<EdgePoint> selectEdges(const std::vector<EdgePoint>& candidates, bool* selectedByFallback = nullptr) const;

    CaliperParams m_params;
};

} // namespace VisionTools
