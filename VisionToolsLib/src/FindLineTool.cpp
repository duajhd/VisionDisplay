#include "VisionTools/FindLineTool.h"

#include <algorithm>
#include <cmath>

namespace VisionTools {

namespace {

constexpr double pi = 3.14159265358979323846;

} // namespace

FindLineTool::FindLineTool()
{
    m_params.caliperParams.selection = EdgeSelection::Strongest;
}

void FindLineTool::setParams(const FindLineParams& params)
{
    m_params = params;
    m_params.minEdgeResponse = std::max(0.0, m_params.minEdgeResponse);
}

const FindLineParams& FindLineTool::params() const
{
    return m_params;
}

FindLineResult FindLineTool::run(const ImageView& image, const LineSearchRegion& region) const
{
    FindLineResult result;
    result.searchRegion = region;

    if (!image.isValid()) {
        result.message = QStringLiteral("Invalid image");
        return result;
    }
    if (region.length <= 0.0 || region.searchLength <= 0.0 || region.caliperCount < 2 || region.caliperWidth < 0.0) {
        result.message = QStringLiteral("Invalid line search region");
        return result;
    }

    result.calipers = generateCalipers(region);

    CaliperTool caliperTool;
    caliperTool.setParams(m_params.caliperParams);

    for (const CaliperRegion& caliper : result.calipers) {
        CaliperResult caliperResult = caliperTool.run(image, caliper);
        if (caliperResult.ok && !caliperResult.selectedEdges.empty()) {
            const EdgePoint& edge = caliperResult.selectedEdges.front();
            if (edge.valid && std::abs(edge.response) >= m_params.minEdgeResponse) {
                result.edgePoints.push_back(edge);
            }
        }
        result.caliperResults.push_back(std::move(caliperResult));
    }

    LineFitter fitter;
    fitter.setParams(m_params.fitParams);
    result.fitResult = fitter.fit(result.edgePoints);
    result.inputPoints = result.fitResult.inputPoints;
    result.inlierPoints = result.fitResult.inlierPoints;
    result.outlierPoints = result.fitResult.outlierPoints;
    result.rmsError = result.fitResult.rmsError;
    result.maxError = result.fitResult.maxError;
    result.score = result.fitResult.score;
    result.ok = result.fitResult.ok;
    result.message = result.ok ? QStringLiteral("OK") : result.fitResult.message;
    return result;
}

std::vector<CaliperRegion> FindLineTool::generateCalipers(const LineSearchRegion& region) const
{
    std::vector<CaliperRegion> calipers;
    if (region.caliperCount <= 0) {
        return calipers;
    }

    calipers.reserve(static_cast<size_t>(region.caliperCount));
    const double angleRad = region.angleDeg * pi / 180.0;
    const double lineDirX = std::cos(angleRad);
    const double lineDirY = std::sin(angleRad);
    const double searchAngleDeg = region.angleDeg + 90.0;
    const double step = region.caliperCount > 1
        ? region.length / static_cast<double>(region.caliperCount - 1)
        : 0.0;

    for (int i = 0; i < region.caliperCount; ++i) {
        const double offset = region.caliperCount > 1 ? -region.length * 0.5 + i * step : 0.0;

        CaliperRegion caliper;
        caliper.centerX = region.centerX + offset * lineDirX;
        caliper.centerY = region.centerY + offset * lineDirY;
        caliper.length = region.searchLength;
        caliper.width = region.caliperWidth;
        caliper.angleDeg = searchAngleDeg;
        calipers.push_back(caliper);
    }

    return calipers;
}

} // namespace VisionTools
