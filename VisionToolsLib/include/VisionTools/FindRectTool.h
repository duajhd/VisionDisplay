#pragma once

#include "VisionTools/CaliperTool.h"
#include "VisionTools/ImageView.h"
#include "VisionTools/LineFitter.h"
#include "VisionTools/RectTypes.h"
#include "VisionTools/VisionTools_global.h"

#include <array>

namespace VisionTools {

class VISIONTOOLS_API FindRectTool
{
public:
    FindRectTool();

    void setParams(const FindRectParams& params);
    const FindRectParams& params() const;

    FindRectResult run(const ImageView& image) const;

private:
    std::array<FindRectEdgeResult, 4> runEdges(const ImageView& image) const;

    FindRectEdgeResult runSingleEdge(const ImageView& image,
                                     RectEdgeId edgeId) const;

    bool buildRectFromLines(const FindRectEdgeResult& top,
                            const FindRectEdgeResult& bottom,
                            const FindRectEdgeResult& left,
                            const FindRectEdgeResult& right,
                            RectModel& outRect,
                            QString& message) const;

    bool validateRect(const RectModel& rect,
                      const FindRectEdgeResult& top,
                      const FindRectEdgeResult& bottom,
                      const FindRectEdgeResult& left,
                      const FindRectEdgeResult& right,
                      QString& message) const;

    FindRectParams m_params;
};

} // namespace VisionTools
