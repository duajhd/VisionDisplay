#include "VisionTools/FindCircleTool.h"

#include <algorithm>
#include <cmath>

namespace VisionTools {

namespace {

constexpr double pi = 3.14159265358979323846;

} // namespace

FindCircleTool::FindCircleTool()
{
    m_params.caliperParams.selection = EdgeSelection::Strongest;
}

void FindCircleTool::setParams(const FindCircleParams& params)
{
    m_params = params;
    m_params.minEdgeResponse = std::max(0.0, m_params.minEdgeResponse);
}

const FindCircleParams& FindCircleTool::params() const
{
    return m_params;
}

FindCircleResult FindCircleTool::run(const ImageView& image, const CircleSearchRegion& region) const
{
    FindCircleResult result;
    result.searchRegion = region;

    if (!image.isValid()) {
        result.message = QStringLiteral("Invalid image");
        return result;
    }
    if (region.radius <= 0.0 || region.searchLength <= 0.0 || region.caliperCount < 3 || region.caliperWidth < 0.0) {
        result.message = QStringLiteral("Invalid circle search region");
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

    CircleFitter fitter;
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

std::vector<CaliperRegion> FindCircleTool::generateCalipers(const CircleSearchRegion& region) const
{
    std::vector<CaliperRegion> calipers;
    if (region.caliperCount <= 0) {
        return calipers;
    }

    calipers.reserve(static_cast<size_t>(region.caliperCount));
    const bool fullCircle = std::abs(region.spanAngleDeg) >= 360.0;
    const double denominator = fullCircle
        ? static_cast<double>(region.caliperCount)
        : static_cast<double>(std::max(1, region.caliperCount - 1));
    const double angleStep = region.spanAngleDeg / denominator;

    for (int i = 0; i < region.caliperCount; ++i) {
        const double angleDeg = region.startAngleDeg + i * angleStep;
        const double angleRad = angleDeg * pi / 180.0;
        const double radialX = std::cos(angleRad);
        const double radialY = std::sin(angleRad);

        CaliperRegion caliper;
        caliper.centerX = region.centerX + region.radius * radialX;
        caliper.centerY = region.centerY + region.radius * radialY;
        caliper.length = region.searchLength;
        caliper.width = region.caliperWidth;
        caliper.angleDeg = region.searchDirection == CircleSearchDirection::Outward
            ? angleDeg
            : angleDeg + 180.0;
        calipers.push_back(caliper);
    }

    return calipers;
}

} // namespace VisionTools
