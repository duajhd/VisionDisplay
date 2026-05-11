#include "ToolResultDisplayAdapter.h"

#include "VisionDisplay/VisionDisplayItem.h"
#include "VisionTools/CaliperTypes.h"
#include "VisionTools/CircleTypes.h"
#include "VisionTools/EllipseTypes.h"
#include "VisionTools/LineTypes.h"

#include <QVariantMap>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace {

constexpr double pi = 3.14159265358979323846;

double normalizedScore(double response)
{
    return std::clamp(std::abs(response) / 50.0, 0.0, 1.0);
}

double caliperRectAngleDeg(const VisionTools::CaliperRegion& region)
{
    return region.angleDeg - 90.0;
}

double lineAngleDeg(const VisionTools::LineModel& line)
{
    double angle = std::atan2(line.y2 - line.y1, line.x2 - line.x1) * 180.0 / pi;
    if (angle < 0.0) {
        angle += 180.0;
    }
    return angle;
}

QVariantMap edgePointMap(const VisionTools::EdgePoint& point, bool outlier)
{
    QVariantMap map;
    map.insert(QStringLiteral("x"), point.x);
    map.insert(QStringLiteral("y"), point.y);
    map.insert(QStringLiteral("nx"), point.nx);
    map.insert(QStringLiteral("ny"), point.ny);
    map.insert(QStringLiteral("score"), normalizedScore(point.response));
    map.insert(QStringLiteral("valid"), point.valid);
    map.insert(QStringLiteral("inlier"), !outlier);
    map.insert(QStringLiteral("outlier"), outlier);
    return map;
}

double ellipseResidual(const VisionTools::EdgePoint& point, const VisionTools::EllipseModel& ellipse)
{
    constexpr double eps = 1.0e-12;
    const double theta = ellipse.angleDeg * pi / 180.0;
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    const double dx = point.x - ellipse.centerX;
    const double dy = point.y - ellipse.centerY;
    const double xLocal = c * dx + s * dy;
    const double yLocal = -s * dx + c * dy;
    const double aa = std::max(1.0, ellipse.radiusA * ellipse.radiusA);
    const double bb = std::max(1.0, ellipse.radiusB * ellipse.radiusB);
    const double f = xLocal * xLocal / aa + yLocal * yLocal / bb - 1.0;
    const double gx = 2.0 * xLocal / aa;
    const double gy = 2.0 * yLocal / bb;
    return f / std::sqrt(gx * gx + gy * gy + eps);
}

} // namespace

void ToolResultDisplayAdapter::showCaliper(VisionDisplay::VisionDisplayItem* display,
                                           const QString& toolId,
                                           const VisionTools::CaliperRegion& region,
                                           const VisionTools::CaliperResult& result)
{
    if (!display) {
        return;
    }

    display->clearToolGraphics(toolId);
    const int bestIndex = selectedEdgeIndex(result.candidates, result.selectedEdges);
    const double response = result.selectedEdges.empty() ? 0.0 : result.selectedEdges.front().response;
    const double score = normalizedScore(response);

    display->addCaliperResult(toolId,
                              region.centerX,
                              region.centerY,
                              region.width,
                              region.length,
                              caliperRectAngleDeg(region),
                              region.angleDeg,
                              edgePointsToVariantList(result.candidates),
                              bestIndex,
                              score,
                              toolId);

    display->addEdgePoints(toolId, edgePointsToVariantList(result.candidates));
    display->addToolStatusText(toolId,
                               region.centerX + 30.0,
                               region.centerY - region.length * 0.55,
                               QStringLiteral("%1 %2 Response=%3")
                                   .arg(toolId)
                                   .arg(result.ok ? QStringLiteral("OK") : QStringLiteral("NG"))
                                   .arg(response, 0, 'f', 2),
                               result.ok);
}

void ToolResultDisplayAdapter::showFindLine(VisionDisplay::VisionDisplayItem* display,
                                            const QString& toolId,
                                            const VisionTools::LineSearchRegion& region,
                                            const VisionTools::FindLineResult& result)
{
    if (!display) {
        return;
    }

    display->clearToolGraphics(toolId);
    display->addFindLineSearchRegion(toolId, region.centerX, region.centerY, region.length, region.searchLength, region.angleDeg);

    QVariantList calipers;
    for (size_t i = 0; i < result.calipers.size(); ++i) {
        const VisionTools::CaliperRegion& caliper = result.calipers[i];
        const VisionTools::CaliperResult* caliperResult = i < result.caliperResults.size() ? &result.caliperResults[i] : nullptr;
        const bool found = caliperResult && caliperResult->ok && !caliperResult->selectedEdges.empty();
        const VisionTools::EdgePoint edge = found ? caliperResult->selectedEdges.front() : VisionTools::EdgePoint {};

        QVariantMap map;
        map.insert(QStringLiteral("id"), QString::number(i));
        map.insert(QStringLiteral("centerX"), caliper.centerX);
        map.insert(QStringLiteral("centerY"), caliper.centerY);
        map.insert(QStringLiteral("width"), caliper.width);
        map.insert(QStringLiteral("height"), caliper.length);
        map.insert(QStringLiteral("angleDeg"), caliperRectAngleDeg(caliper));
        map.insert(QStringLiteral("searchDirectionAngleDeg"), caliper.angleDeg);
        map.insert(QStringLiteral("found"), found);
        map.insert(QStringLiteral("edgeX"), edge.x);
        map.insert(QStringLiteral("edgeY"), edge.y);
        map.insert(QStringLiteral("score"), normalizedScore(edge.response));
        calipers.push_back(map);
    }
    display->addLineCalipers(toolId, calipers);

    display->addEdgePoints(toolId, edgePointsToVariantList(result.fitResult.inlierPoints));
    display->addEdgePoints(toolId, edgePointsToVariantList(result.fitResult.outlierPoints, true));

    if (result.fitResult.ok) {
        const VisionTools::LineModel& line = result.fitResult.line;
        display->addFittedLineResult(toolId,
                                     line.x1,
                                     line.y1,
                                     line.x2,
                                     line.y2,
                                     lineAngleDeg(line),
                                     result.fitResult.score,
                                     result.fitResult.rmsError,
                                     QStringLiteral("FindLine"));
    }

    display->addToolStatusText(toolId,
                               region.centerX - region.length * 0.45,
                               region.centerY - region.searchLength * 0.75,
                               QStringLiteral("%1 %2 Points=%3 RMS=%4 Score=%5")
                                   .arg(toolId)
                                   .arg(result.ok ? QStringLiteral("OK") : QStringLiteral("NG"))
                                   .arg(static_cast<int>(result.edgePoints.size()))
                                   .arg(result.fitResult.rmsError, 0, 'f', 3)
                                   .arg(result.fitResult.score, 0, 'f', 2),
                               result.ok);
}

void ToolResultDisplayAdapter::showFindCircle(VisionDisplay::VisionDisplayItem* display,
                                              const QString& toolId,
                                              const VisionTools::CircleSearchRegion& region,
                                              const VisionTools::FindCircleResult& result)
{
    if (!display) {
        return;
    }

    display->clearToolGraphics(toolId);
    display->addExpectedArc(toolId, region.centerX, region.centerY, region.radius, region.startAngleDeg, region.spanAngleDeg);
    display->addCircleSearchAnnulus(toolId,
                                    region.centerX,
                                    region.centerY,
                                    std::max(0.0, region.radius - region.searchLength * 0.5),
                                    region.radius + region.searchLength * 0.5,
                                    region.startAngleDeg,
                                    region.spanAngleDeg,
                                    region.searchDirection == VisionTools::CircleSearchDirection::Outward ? 0.0 : 180.0);

    QVariantList calipers;
    for (size_t i = 0; i < result.calipers.size(); ++i) {
        const VisionTools::CaliperRegion& caliper = result.calipers[i];
        const VisionTools::CaliperResult* caliperResult = i < result.caliperResults.size() ? &result.caliperResults[i] : nullptr;
        const bool found = caliperResult && caliperResult->ok && !caliperResult->selectedEdges.empty();
        const VisionTools::EdgePoint edge = found ? caliperResult->selectedEdges.front() : VisionTools::EdgePoint {};

        QVariantMap map;
        map.insert(QStringLiteral("id"), QString::number(i));
        map.insert(QStringLiteral("centerX"), caliper.centerX);
        map.insert(QStringLiteral("centerY"), caliper.centerY);
        map.insert(QStringLiteral("width"), caliper.width);
        map.insert(QStringLiteral("height"), caliper.length);
        map.insert(QStringLiteral("angleDeg"), caliperRectAngleDeg(caliper));
        map.insert(QStringLiteral("searchDirectionAngleDeg"), caliper.angleDeg);
        map.insert(QStringLiteral("found"), found);
        map.insert(QStringLiteral("edgeX"), edge.x);
        map.insert(QStringLiteral("edgeY"), edge.y);
        map.insert(QStringLiteral("score"), normalizedScore(edge.response));
        calipers.push_back(map);
    }
    display->addRadialCalipers(toolId, calipers);
    display->addEdgePoints(toolId, edgePointsToVariantList(result.fitResult.inlierPoints));
    display->addEdgePoints(toolId, edgePointsToVariantList(result.fitResult.outlierPoints, true));

    if (result.fitResult.ok) {
        const VisionTools::CircleModel& circle = result.fitResult.circle;
        display->addFittedCircleResult(toolId,
                                       circle.centerX,
                                       circle.centerY,
                                       circle.radius,
                                       result.fitResult.score,
                                       result.fitResult.rmsError,
                                       QStringLiteral("FindCircle"));
    }

    display->addToolStatusText(toolId,
                               region.centerX - region.radius,
                               region.centerY - region.radius - 35.0,
                               QStringLiteral("%1 %2 Points=%3 R=%4 RMS=%5 Score=%6")
                                   .arg(toolId)
                                   .arg(result.ok ? QStringLiteral("OK") : QStringLiteral("NG"))
                                   .arg(static_cast<int>(result.edgePoints.size()))
                                   .arg(result.fitResult.circle.radius, 0, 'f', 2)
                                   .arg(result.fitResult.rmsError, 0, 'f', 3)
                                   .arg(result.fitResult.score, 0, 'f', 2),
                               result.ok);
}

void ToolResultDisplayAdapter::showFindEllipse(VisionDisplay::VisionDisplayItem* display,
                                               const QString& toolId,
                                               const VisionTools::FindEllipseResult& result)
{
    if (!display) {
        return;
    }

    display->clearToolGraphics(toolId);
    display->addExpectedEllipse(toolId,
                                result.expectedEllipse.centerX,
                                result.expectedEllipse.centerY,
                                result.expectedEllipse.radiusA,
                                result.expectedEllipse.radiusB,
                                result.expectedEllipse.angleDeg,
                                0.0,
                                360.0);

    QVariantList calipers;
    for (size_t i = 0; i < result.calipers.size(); ++i) {
        const VisionTools::CaliperRegion& caliper = result.calipers[i];
        const VisionTools::EdgePoint edge = i < result.caliperHitPoints.size() ? result.caliperHitPoints[i] : VisionTools::EdgePoint {};
        QVariantMap map;
        map.insert(QStringLiteral("id"), QString::number(i));
        map.insert(QStringLiteral("centerX"), caliper.centerX);
        map.insert(QStringLiteral("centerY"), caliper.centerY);
        map.insert(QStringLiteral("width"), caliper.width);
        map.insert(QStringLiteral("height"), caliper.length);
        map.insert(QStringLiteral("angleDeg"), caliperRectAngleDeg(caliper));
        map.insert(QStringLiteral("searchDirectionAngleDeg"), caliper.angleDeg);
        map.insert(QStringLiteral("found"), edge.valid);
        map.insert(QStringLiteral("edgeX"), edge.x);
        map.insert(QStringLiteral("edgeY"), edge.y);
        map.insert(QStringLiteral("score"), normalizedScore(edge.response));
        calipers.push_back(map);
    }
    display->addLineCalipers(toolId, calipers);

    display->addEdgePoints(toolId, edgePointsToVariantList(result.inlierPoints));
    display->addEdgePoints(toolId, edgePointsToVariantList(result.outlierPoints, true));

    const VisionTools::EdgePoint* maxResidualPoint = nullptr;
    double maxResidual = -1.0;
    for (const VisionTools::EdgePoint& point : result.inputPoints) {
        const double residual = std::abs(ellipseResidual(point, result.fittedEllipse));
        if (residual > maxResidual) {
            maxResidual = residual;
            maxResidualPoint = &point;
        }
    }
    if (maxResidualPoint) {
        display->addToolPointMarker(toolId, QStringLiteral("max_residual"), maxResidualPoint->x, maxResidualPoint->y, 255, 40, 255, 16.0);
    }

    display->addFittedEllipseResult(toolId,
                                    result.fittedEllipse.centerX,
                                    result.fittedEllipse.centerY,
                                    result.fittedEllipse.radiusA,
                                    result.fittedEllipse.radiusB,
                                    result.fittedEllipse.angleDeg,
                                    result.score,
                                    result.rmsError,
                                    QStringLiteral("FindEllipse"));

    display->addToolStatusText(toolId,
                               result.expectedEllipse.centerX - result.expectedEllipse.radiusA,
                               result.expectedEllipse.centerY - result.expectedEllipse.radiusB - 42.0,
                               QStringLiteral("%1 %2 RMS=%3 Max=%4 InlierRatio=%5 In=%6 Out=%7")
                                   .arg(toolId)
                                   .arg(result.ok ? QStringLiteral("OK") : QStringLiteral("NG"))
                                   .arg(result.diagnostics.rmsError, 0, 'f', 3)
                                   .arg(result.diagnostics.maxError, 0, 'f', 3)
                                   .arg(result.diagnostics.inlierRatio, 0, 'f', 2)
                                   .arg(result.diagnostics.inlierCount)
                                   .arg(result.diagnostics.outlierCount),
                               result.ok);
}

QVariantList ToolResultDisplayAdapter::edgePointsToVariantList(const std::vector<VisionTools::EdgePoint>& points, bool outlier)
{
    QVariantList list;
    list.reserve(static_cast<int>(points.size()));
    for (const VisionTools::EdgePoint& point : points) {
        list.push_back(edgePointMap(point, outlier));
    }
    return list;
}

int ToolResultDisplayAdapter::selectedEdgeIndex(const std::vector<VisionTools::EdgePoint>& candidates,
                                                const std::vector<VisionTools::EdgePoint>& selectedEdges)
{
    if (candidates.empty() || selectedEdges.empty()) {
        return -1;
    }

    const VisionTools::EdgePoint& selected = selectedEdges.front();
    int bestIndex = -1;
    double bestDistance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < candidates.size(); ++i) {
        const double dx = candidates[i].x - selected.x;
        const double dy = candidates[i].y - selected.y;
        const double distance = dx * dx + dy * dy;
        if (distance < bestDistance) {
            bestDistance = distance;
            bestIndex = static_cast<int>(i);
        }
    }
    return bestIndex;
}
