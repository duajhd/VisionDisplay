#include "VisionTools/FindCircleTool.h"

#include <QStringList>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace VisionTools {

namespace {

constexpr double pi = 3.14159265358979323846;

bool sameEdgePoint(const EdgePoint& a, const EdgePoint& b)
{
    constexpr double epsilon = 1.0e-6;
    return std::abs(a.x - b.x) <= epsilon
        && std::abs(a.y - b.y) <= epsilon
        && std::abs(a.position1D - b.position1D) <= epsilon;
}

double normalizedAngleDeg(double angleDeg)
{
    while (angleDeg < 0.0) {
        angleDeg += 360.0;
    }
    while (angleDeg >= 360.0) {
        angleDeg -= 360.0;
    }
    return angleDeg;
}

double expectedPosition1DForCaliper(const CircleSearchRegion& region, const CaliperRegion& caliper)
{
    const double radialAngleDeg = region.searchDirection == CircleSearchDirection::Outward
        ? caliper.angleDeg
        : caliper.angleDeg - 180.0;
    const double radialAngleRad = normalizedAngleDeg(radialAngleDeg) * pi / 180.0;
    const double expectedX = region.centerX + region.radius * std::cos(radialAngleRad);
    const double expectedY = region.centerY + region.radius * std::sin(radialAngleRad);
    const double searchAngleRad = caliper.angleDeg * pi / 180.0;
    const double dirX = std::cos(searchAngleRad);
    const double dirY = std::sin(searchAngleRad);
    return (expectedX - caliper.centerX) * dirX + (expectedY - caliper.centerY) * dirY;
}

QString diagnosticSummaryText(const FindCircleDiagnosticSummary& diagnostics)
{
    return QStringLiteral(
               "FindCircle Summary:\n"
               "- caliperCount=%1\n"
               "- caliperHitCount=%2\n"
               "- selectedEdgeCount=%3\n"
               "- withinExpectedCount=%4\n"
               "- rejectedByExpectedWindowCount=%5\n"
               "- fitInputPointsCount=%6\n"
               "- initialFitPointCount=%7\n"
               "- inlierCount=%8\n"
               "- outlierCount=%9\n"
               "- finalFitPointCount=%10\n"
               "- ok=%11")
        .arg(diagnostics.caliperCount)
        .arg(diagnostics.caliperHitCount)
        .arg(diagnostics.selectedEdgeCount)
        .arg(diagnostics.withinExpectedCount)
        .arg(diagnostics.rejectedByExpectedWindowCount)
        .arg(diagnostics.fitInputPointsCount)
        .arg(diagnostics.initialFitPointCount)
        .arg(diagnostics.inlierCount)
        .arg(diagnostics.outlierCount)
        .arg(diagnostics.finalFitPointCount)
        .arg(diagnostics.ok ? QStringLiteral("true") : QStringLiteral("false"));
}

QString caliperDiagnosticText(const FindCircleCaliperDiagnostic& diagnostic)
{
    return QStringLiteral("FindCircle Caliper index=%1 center=(%2,%3) searchDirectionAngleDeg=%4 "
                          "selected=(%5,%6) response=%7 position1D=%8 expectedPosition1D=%9 "
                          "maxPositionDeviation=%10 withinExpected=%11 selectedByFallback=%12 "
                          "acceptedForFit=%13 rejectedReason=%14")
        .arg(diagnostic.index)
        .arg(diagnostic.centerX, 0, 'f', 3)
        .arg(diagnostic.centerY, 0, 'f', 3)
        .arg(diagnostic.searchDirectionAngleDeg, 0, 'f', 3)
        .arg(diagnostic.selectedX, 0, 'f', 3)
        .arg(diagnostic.selectedY, 0, 'f', 3)
        .arg(diagnostic.response, 0, 'f', 3)
        .arg(diagnostic.position1D, 0, 'f', 3)
        .arg(diagnostic.expectedPosition1D, 0, 'f', 3)
        .arg(diagnostic.maxPositionDeviation, 0, 'f', 3)
        .arg(diagnostic.withinExpected ? QStringLiteral("true") : QStringLiteral("false"))
        .arg(diagnostic.selectedByFallback ? QStringLiteral("true") : QStringLiteral("false"))
        .arg(diagnostic.acceptedForFit ? QStringLiteral("true") : QStringLiteral("false"))
        .arg(diagnostic.rejectedReason);
}

} // namespace

FindCircleTool::FindCircleTool()
{
    m_params.caliperParams.selection = EdgeSelection::NearestToExpected;
    m_params.caliperParams.expectedPosition1D = 0.0;
    m_params.caliperParams.allowFallbackSelection = false;
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
    result.diagnostics.caliperCount = static_cast<int>(result.calipers.size());
    result.candidateEdgePoints.reserve(result.calipers.size());
    result.edgePoints.reserve(result.calipers.size());
    result.rejectedEdgePoints.reserve(result.calipers.size());

    CaliperParams caliperParams = m_params.caliperParams;
    caliperParams.selection = EdgeSelection::NearestToExpected;
    caliperParams.allowFallbackSelection = false;
    const double expectedTolerance = m_params.circleExpectedPositionTolerance > 0.0
        ? m_params.circleExpectedPositionTolerance
        : std::max(caliperParams.maxPositionDeviation, region.searchLength * 0.5);
    caliperParams.maxPositionDeviation = std::max(0.0, expectedTolerance);

    CaliperTool caliperTool;

    for (size_t i = 0; i < result.calipers.size(); ++i) {
        const CaliperRegion& caliper = result.calipers[i];
        caliperParams.expectedPosition1D = expectedPosition1DForCaliper(region, caliper);
        caliperTool.setParams(caliperParams);

        CaliperResult caliperResult = caliperTool.run(image, caliper);
        FindCircleCaliperDiagnostic diagnostic;
        diagnostic.index = static_cast<int>(i);
        diagnostic.centerX = caliper.centerX;
        diagnostic.centerY = caliper.centerY;
        diagnostic.searchDirectionAngleDeg = caliper.angleDeg;
        diagnostic.expectedPosition1D = caliperParams.expectedPosition1D;
        diagnostic.maxPositionDeviation = caliperParams.maxPositionDeviation;
        diagnostic.selectedByFallback = caliperResult.selectedByFallback;

        for (const EdgePoint& candidate : caliperResult.candidates) {
            if (candidate.valid) {
                result.candidateEdgePoints.push_back(candidate);
            }
        }

        if (caliperResult.ok && !caliperResult.selectedEdges.empty()) {
            const EdgePoint edge = caliperResult.selectedEdges.front();
            diagnostic.hasSelectedEdge = edge.valid;
            diagnostic.selectedX = edge.x;
            diagnostic.selectedY = edge.y;
            diagnostic.response = edge.response;
            diagnostic.position1D = edge.position1D;
            ++result.diagnostics.caliperHitCount;
            ++result.diagnostics.selectedEdgeCount;

            const bool withinExpected = std::abs(edge.position1D - caliperParams.expectedPosition1D)
                <= caliperParams.maxPositionDeviation;
            diagnostic.withinExpected = withinExpected;
            if (withinExpected) {
                ++result.diagnostics.withinExpectedCount;
            }

            if (!edge.valid) {
                diagnostic.rejectedReason = QStringLiteral("selected edge is invalid");
            } else if (!withinExpected) {
                ++result.diagnostics.rejectedByExpectedWindowCount;
                diagnostic.rejectedReason = QStringLiteral("outside expected position window");
            } else if (caliperResult.selectedByFallback && !m_params.caliperParams.allowFallbackSelection) {
                diagnostic.rejectedReason = QStringLiteral("selected by fallback while fallback selection is disabled");
            } else if (std::abs(edge.response) < m_params.minEdgeResponse) {
                diagnostic.rejectedReason = QStringLiteral("response below minEdgeResponse");
            } else {
                diagnostic.acceptedForFit = true;
                result.edgePoints.push_back(edge);
            }

            if (edge.valid && !diagnostic.acceptedForFit) {
                result.rejectedEdgePoints.push_back(edge);
            }
        } else {
            diagnostic.rejectedReason = caliperResult.message.isEmpty()
                ? QStringLiteral("no selected edge")
                : caliperResult.message;
            for (const EdgePoint& candidate : caliperResult.candidates) {
                if (candidate.valid) {
                    result.rejectedEdgePoints.push_back(candidate);
                }
            }
        }
        CaliperResult storedCaliperResult;
        storedCaliperResult.ok = caliperResult.ok;
        storedCaliperResult.candidates = caliperResult.candidates;
        storedCaliperResult.selectedEdges = caliperResult.selectedEdges;
        storedCaliperResult.selectedByFallback = caliperResult.selectedByFallback;
        result.caliperResults.push_back(std::move(storedCaliperResult));
    }

    result.diagnostics.fitInputPointsCount = static_cast<int>(result.edgePoints.size());
    result.diagnostics.initialFitPointCount = result.diagnostics.fitInputPointsCount;
    if (result.edgePoints.size() < 3) {
        result.message = QStringLiteral("Not enough accepted edge points for circle fitting. fitInputPointsCount=%1")
                             .arg(static_cast<int>(result.edgePoints.size()));
        result.diagnostics.ok = false;
        return result;
    }

    CircleFitter fitter;
    fitter.setParams(m_params.fitParams);
    const CircleFitResult fitResult = fitter.fit(result.edgePoints);
    result.fitResult.ok = fitResult.ok;
    result.fitResult.circle = fitResult.circle;
    result.fitResult.inputPoints = fitResult.inputPoints;
    result.fitResult.inlierPoints = fitResult.inlierPoints;
    result.fitResult.outlierPoints = fitResult.outlierPoints;
    result.fitResult.rmsError = fitResult.rmsError;
    result.fitResult.maxError = fitResult.maxError;
    result.fitResult.score = fitResult.score;
    result.inputPoints = fitResult.inputPoints;
    result.inlierPoints = fitResult.inlierPoints;
    result.outlierPoints = fitResult.outlierPoints;
    result.rmsError = fitResult.rmsError;
    result.maxError = fitResult.maxError;
    result.score = fitResult.score;
    result.ok = fitResult.ok;
    result.diagnostics.inlierCount = static_cast<int>(result.inlierPoints.size());
    result.diagnostics.outlierCount = static_cast<int>(result.outlierPoints.size());
    result.diagnostics.finalFitPointCount = result.diagnostics.inlierCount;
    result.diagnostics.ok = result.ok;
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
