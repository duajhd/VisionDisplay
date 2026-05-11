#include "VisionTools/FindRectTool.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace VisionTools {

namespace {

constexpr double pi = 3.14159265358979323846;

struct EdgeGeometry
{
    double centerX = 0.0;
    double centerY = 0.0;
    double lineAngleDeg = 0.0;
    double searchAngleDeg = 0.0;
    double spanLength = 0.0;
};

double degToRad(double degrees)
{
    return degrees * pi / 180.0;
}

double radToDeg(double radians)
{
    return radians * 180.0 / pi;
}

double normalizeAngle180(double angleDeg)
{
    double value = std::fmod(angleDeg, 180.0);
    if (value < 0.0) {
        value += 180.0;
    }
    return value;
}

double angleDifference180(double aDeg, double bDeg)
{
    double diff = std::abs(normalizeAngle180(aDeg) - normalizeAngle180(bDeg));
    if (diff > 90.0) {
        diff = 180.0 - diff;
    }
    return diff;
}

double lineAngleDeg(const LineModel& line)
{
    return normalizeAngle180(radToDeg(std::atan2(line.y2 - line.y1, line.x2 - line.x1)));
}

double distance(const Point2D& a, const Point2D& b)
{
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
}

bool lineIntersection(const LineModel& a, const LineModel& b, Point2D* outPoint)
{
    const double det = a.nx * b.ny - b.nx * a.ny;
    if (std::abs(det) <= 1.0e-9) {
        return false;
    }

    if (outPoint) {
        outPoint->x = (a.rho * b.ny - b.rho * a.ny) / det;
        outPoint->y = (a.nx * b.rho - b.nx * a.rho) / det;
    }
    return true;
}

EdgeGeometry edgeGeometry(const FindRectParams& params, RectEdgeId edgeId)
{
    const double angleRad = degToRad(params.angleDeg);
    const double ux = std::cos(angleRad);
    const double uy = std::sin(angleRad);
    const double vx = -std::sin(angleRad);
    const double vy = std::cos(angleRad);
    const bool outsideToInside = params.searchDirection == RectSearchDirection::FromOutsideToInside;

    EdgeGeometry geometry;
    switch (edgeId) {
    case RectEdgeId::Top:
        geometry.centerX = params.centerX - vx * params.expectedHeight * 0.5;
        geometry.centerY = params.centerY - vy * params.expectedHeight * 0.5;
        geometry.lineAngleDeg = params.angleDeg;
        geometry.searchAngleDeg = params.angleDeg + (outsideToInside ? 90.0 : -90.0);
        geometry.spanLength = params.expectedWidth;
        break;
    case RectEdgeId::Bottom:
        geometry.centerX = params.centerX + vx * params.expectedHeight * 0.5;
        geometry.centerY = params.centerY + vy * params.expectedHeight * 0.5;
        geometry.lineAngleDeg = params.angleDeg;
        geometry.searchAngleDeg = params.angleDeg + (outsideToInside ? -90.0 : 90.0);
        geometry.spanLength = params.expectedWidth;
        break;
    case RectEdgeId::Left:
        geometry.centerX = params.centerX - ux * params.expectedWidth * 0.5;
        geometry.centerY = params.centerY - uy * params.expectedWidth * 0.5;
        geometry.lineAngleDeg = params.angleDeg + 90.0;
        geometry.searchAngleDeg = params.angleDeg + (outsideToInside ? 0.0 : 180.0);
        geometry.spanLength = params.expectedHeight;
        break;
    case RectEdgeId::Right:
        geometry.centerX = params.centerX + ux * params.expectedWidth * 0.5;
        geometry.centerY = params.centerY + uy * params.expectedWidth * 0.5;
        geometry.lineAngleDeg = params.angleDeg + 90.0;
        geometry.searchAngleDeg = params.angleDeg + (outsideToInside ? 180.0 : 0.0);
        geometry.spanLength = params.expectedHeight;
        break;
    }
    return geometry;
}

const char* edgeName(RectEdgeId edgeId)
{
    switch (edgeId) {
    case RectEdgeId::Top: return "top";
    case RectEdgeId::Bottom: return "bottom";
    case RectEdgeId::Left: return "left";
    case RectEdgeId::Right: return "right";
    }
    return "edge";
}

double rectMaxError(const FindRectEdgeResult& top,
                    const FindRectEdgeResult& bottom,
                    const FindRectEdgeResult& left,
                    const FindRectEdgeResult& right)
{
    double maxError = 0.0;
    for (const FindRectEdgeResult* edge : {&top, &bottom, &left, &right}) {
        for (const EdgePoint& point : edge->inlierPoints) {
            const double error = std::abs(edge->fittedLine.nx * point.x
                                          + edge->fittedLine.ny * point.y
                                          - edge->fittedLine.rho);
            maxError = std::max(maxError, error);
        }
    }
    return maxError;
}

} // namespace

FindRectTool::FindRectTool() = default;

void FindRectTool::setParams(const FindRectParams& params)
{
    m_params = params;
    m_params.expectedWidth = std::max(1.0, m_params.expectedWidth);
    m_params.expectedHeight = std::max(1.0, m_params.expectedHeight);
    m_params.calipersPerEdge = std::max(2, m_params.calipersPerEdge);
    m_params.searchLength = std::max(1.0, m_params.searchLength);
    m_params.projectionWidth = std::max(0.0, m_params.projectionWidth);
    m_params.minResponse = std::max(0.0, m_params.minResponse);
    m_params.maxLineResidual = std::max(0.0, m_params.maxLineResidual);
    m_params.maxParallelAngleErrorDeg = std::max(0.0, m_params.maxParallelAngleErrorDeg);
    m_params.maxOrthogonalAngleErrorDeg = std::max(0.0, m_params.maxOrthogonalAngleErrorDeg);
    m_params.maxSizeErrorRatio = std::max(0.0, m_params.maxSizeErrorRatio);
}

const FindRectParams& FindRectTool::params() const
{
    return m_params;
}

FindRectResult FindRectTool::run(const ImageView& image) const
{
    FindRectResult result;
    if (!image.isValid()) {
        result.message = QStringLiteral("Invalid image");
        return result;
    }

    const std::array<FindRectEdgeResult, 4> edges = runEdges(image);
    result.top = edges[0];
    result.bottom = edges[1];
    result.left = edges[2];
    result.right = edges[3];

    for (const FindRectEdgeResult* edge : {&result.top, &result.bottom, &result.left, &result.right}) {
        if (!edge->ok) {
            result.message = QStringLiteral("%1 edge failed: %2")
                                 .arg(QString::fromLatin1(edgeName(edge->edgeId)), edge->message);
            return result;
        }
    }

    if (!buildRectFromLines(result.top, result.bottom, result.left, result.right, result.rect, result.message)) {
        return result;
    }
    if (!validateRect(result.rect, result.top, result.bottom, result.left, result.right, result.message)) {
        return result;
    }

    double weightedSumSq = 0.0;
    int inlierCount = 0;
    int inputCount = 0;
    double scoreSum = 0.0;
    for (const FindRectEdgeResult* edge : {&result.top, &result.bottom, &result.left, &result.right}) {
        weightedSumSq += edge->rmsError * edge->rmsError * static_cast<double>(edge->inlierPoints.size());
        inlierCount += static_cast<int>(edge->inlierPoints.size());
        inputCount += static_cast<int>(edge->inputPoints.size());
        scoreSum += edge->score;
    }

    result.rmsError = inlierCount > 0 ? std::sqrt(weightedSumSq / static_cast<double>(inlierCount)) : 0.0;
    result.maxError = rectMaxError(result.top, result.bottom, result.left, result.right);
    const double inlierRatio = inputCount > 0 ? static_cast<double>(inlierCount) / static_cast<double>(inputCount) : 0.0;
    const double rmsScale = std::max(1.0, m_params.maxLineResidual);
    const double rmsScore = 1.0 / (1.0 + result.rmsError / rmsScale);
    result.score = std::clamp(0.45 * inlierRatio + 0.35 * (scoreSum * 0.25) + 0.20 * rmsScore, 0.0, 1.0);
    result.ok = true;
    result.message = QStringLiteral("OK");
    return result;
}

std::array<FindRectEdgeResult, 4> FindRectTool::runEdges(const ImageView& image) const
{
    return {
        runSingleEdge(image, RectEdgeId::Top),
        runSingleEdge(image, RectEdgeId::Bottom),
        runSingleEdge(image, RectEdgeId::Left),
        runSingleEdge(image, RectEdgeId::Right)
    };
}

FindRectEdgeResult FindRectTool::runSingleEdge(const ImageView& image, RectEdgeId edgeId) const
{
    FindRectEdgeResult result;
    result.edgeId = edgeId;

    const EdgeGeometry geometry = edgeGeometry(m_params, edgeId);
    const double lineAngleRad = degToRad(geometry.lineAngleDeg);
    const double lineDirX = std::cos(lineAngleRad);
    const double lineDirY = std::sin(lineAngleRad);
    const double step = m_params.calipersPerEdge > 1
        ? geometry.spanLength / static_cast<double>(m_params.calipersPerEdge - 1)
        : 0.0;

    result.calipers.reserve(static_cast<size_t>(m_params.calipersPerEdge));
    for (int i = 0; i < m_params.calipersPerEdge; ++i) {
        const double offset = m_params.calipersPerEdge > 1 ? -geometry.spanLength * 0.5 + i * step : 0.0;
        CaliperRegion caliper;
        caliper.centerX = geometry.centerX + offset * lineDirX;
        caliper.centerY = geometry.centerY + offset * lineDirY;
        caliper.length = m_params.searchLength;
        caliper.width = m_params.projectionWidth;
        caliper.angleDeg = geometry.searchAngleDeg;
        result.calipers.push_back(caliper);
    }

    CaliperParams caliperParams;
    caliperParams.sampleCount = 101;
    caliperParams.projectionCount = std::max(1, static_cast<int>(std::round(m_params.projectionWidth / 2.0)));
    caliperParams.smoothingSigma = 1.0;
    caliperParams.minResponse = m_params.minResponse;
    caliperParams.polarity = m_params.polarity;
    caliperParams.selection = m_params.selection;
    caliperParams.enableSubpixel = true;

    CaliperTool caliperTool;
    caliperTool.setParams(caliperParams);
    for (const CaliperRegion& caliper : result.calipers) {
        const CaliperResult caliperResult = caliperTool.run(image, caliper);
        if (caliperResult.ok && !caliperResult.selectedEdges.empty()) {
            const EdgePoint& edge = caliperResult.selectedEdges.front();
            if (edge.valid && std::abs(edge.response) >= m_params.minResponse) {
                result.inputPoints.push_back(edge);
            }
        }
    }

    LineFitParams fitParams;
    fitParams.maxResidual = m_params.maxLineResidual;
    fitParams.minInlierCount = std::max(2, m_params.calipersPerEdge / 2);
    fitParams.enableRansac = true;
    fitParams.ransacIterations = 120;
    fitParams.ransacResidual = m_params.maxLineResidual;
    fitParams.enableHuber = true;
    fitParams.huberDelta = std::max(1.0, m_params.maxLineResidual * 0.75);
    fitParams.robustIterations = 3;

    LineFitter fitter;
    fitter.setParams(fitParams);
    const LineFitResult fit = fitter.fit(result.inputPoints);
    result.fittedLine = fit.line;
    result.inlierPoints = fit.inlierPoints;
    result.outlierPoints = fit.outlierPoints;
    result.rmsError = fit.rmsError;
    result.score = fit.score;
    result.ok = fit.ok;
    result.message = fit.message;
    return result;
}

bool FindRectTool::buildRectFromLines(const FindRectEdgeResult& top,
                                      const FindRectEdgeResult& bottom,
                                      const FindRectEdgeResult& left,
                                      const FindRectEdgeResult& right,
                                      RectModel& outRect,
                                      QString& message) const
{
    if (!lineIntersection(top.fittedLine, left.fittedLine, &outRect.topLeft)
        || !lineIntersection(top.fittedLine, right.fittedLine, &outRect.topRight)
        || !lineIntersection(bottom.fittedLine, right.fittedLine, &outRect.bottomRight)
        || !lineIntersection(bottom.fittedLine, left.fittedLine, &outRect.bottomLeft)) {
        message = QStringLiteral("Could not intersect fitted rectangle lines");
        return false;
    }

    outRect.centerX = (outRect.topLeft.x + outRect.topRight.x + outRect.bottomRight.x + outRect.bottomLeft.x) * 0.25;
    outRect.centerY = (outRect.topLeft.y + outRect.topRight.y + outRect.bottomRight.y + outRect.bottomLeft.y) * 0.25;
    outRect.width = 0.5 * (distance(outRect.topLeft, outRect.topRight)
                           + distance(outRect.bottomLeft, outRect.bottomRight));
    outRect.height = 0.5 * (distance(outRect.topLeft, outRect.bottomLeft)
                            + distance(outRect.topRight, outRect.bottomRight));
    outRect.angleDeg = normalizeAngle180(radToDeg(std::atan2(outRect.topRight.y - outRect.topLeft.y,
                                                             outRect.topRight.x - outRect.topLeft.x)));
    return true;
}

bool FindRectTool::validateRect(const RectModel& rect,
                                const FindRectEdgeResult& top,
                                const FindRectEdgeResult& bottom,
                                const FindRectEdgeResult& left,
                                const FindRectEdgeResult& right,
                                QString& message) const
{
    if (rect.width <= 0.0 || rect.height <= 0.0) {
        message = QStringLiteral("Invalid rectangle size");
        return false;
    }

    const double topAngle = lineAngleDeg(top.fittedLine);
    const double bottomAngle = lineAngleDeg(bottom.fittedLine);
    const double leftAngle = lineAngleDeg(left.fittedLine);
    const double rightAngle = lineAngleDeg(right.fittedLine);

    if (m_params.enforceParallel) {
        const double horizontalError = angleDifference180(topAngle, bottomAngle);
        const double verticalError = angleDifference180(leftAngle, rightAngle);
        if (horizontalError > m_params.maxParallelAngleErrorDeg || verticalError > m_params.maxParallelAngleErrorDeg) {
            message = QStringLiteral("Opposite edges are not parallel enough");
            return false;
        }
    }

    if (m_params.enforceOrthogonal) {
        const double errors[] = {
            std::abs(90.0 - angleDifference180(topAngle, leftAngle)),
            std::abs(90.0 - angleDifference180(topAngle, rightAngle)),
            std::abs(90.0 - angleDifference180(bottomAngle, leftAngle)),
            std::abs(90.0 - angleDifference180(bottomAngle, rightAngle))
        };
        const double maxError = *std::max_element(std::begin(errors), std::end(errors));
        if (maxError > m_params.maxOrthogonalAngleErrorDeg) {
            message = QStringLiteral("Adjacent edges are not orthogonal enough");
            return false;
        }
    }

    const double widthRatio = std::abs(rect.width - m_params.expectedWidth) / std::max(1.0, m_params.expectedWidth);
    const double heightRatio = std::abs(rect.height - m_params.expectedHeight) / std::max(1.0, m_params.expectedHeight);
    if (widthRatio > m_params.maxSizeErrorRatio || heightRatio > m_params.maxSizeErrorRatio) {
        message = QStringLiteral("Detected rectangle size differs too much from expected size");
        return false;
    }

    return true;
}

} // namespace VisionTools
