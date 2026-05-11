#pragma once

#include "VisionTools/CaliperTool.h"
#include "VisionTools/EllipseFitter.h"
#include "VisionTools/EllipseTypes.h"
#include "VisionTools/ImageView.h"
#include "VisionTools/VisionTools_global.h"

#include <vector>

namespace VisionTools {

class VISIONTOOLS_API FindEllipseTool
{
public:
    FindEllipseTool();

    void setParams(const FindEllipseParams& params);
    const FindEllipseParams& params() const;

    FindEllipseResult run(const ImageView& image) const;

private:
    std::vector<CaliperRegion> generateCalipers() const;

    EdgePoint runSingleCaliper(const ImageView& image,
                               const CaliperRegion& caliper) const;

    EllipseModel buildInitialEllipse(const std::vector<EdgePoint>& points,
                                     QString& message) const;

    void splitInliersOutliers(const std::vector<EdgePoint>& points,
                              const EllipseModel& ellipse,
                              std::vector<EdgePoint>& inliers,
                              std::vector<EdgePoint>& outliers) const;

    double computeScore(const FindEllipseResult& result) const;

    FindEllipseParams m_params;
};

} // namespace VisionTools
