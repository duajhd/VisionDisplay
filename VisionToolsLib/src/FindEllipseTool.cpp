#include "VisionTools/FindEllipseTool.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace VisionTools {

namespace {

constexpr double pi = 3.14159265358979323846;

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

EllipseModel normalizedExpected(const FindEllipseParams& params)
{
    EllipseModel ellipse;
    ellipse.centerX = params.centerX;
    ellipse.centerY = params.centerY;
    ellipse.radiusA = std::max(1.0, params.radiusA);
    ellipse.radiusB = std::max(1.0, params.radiusB);
    ellipse.angleDeg = params.angleDeg;
    if (ellipse.radiusA < ellipse.radiusB) {
        std::swap(ellipse.radiusA, ellipse.radiusB);
        ellipse.angleDeg += 90.0;
    }
    ellipse.angleDeg = normalizeAngle180(ellipse.angleDeg);
    return ellipse;
}

double residualApproxGeometric(const EdgePoint& point, const EllipseModel& ellipse)
{
    constexpr double eps = 1.0e-12;
    const double a = std::max(ellipse.radiusA, 1.0e-6);
    const double b = std::max(ellipse.radiusB, 1.0e-6);
    const double theta = degToRad(ellipse.angleDeg);
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    const double dx = point.x - ellipse.centerX;
    const double dy = point.y - ellipse.centerY;
    const double xLocal = c * dx + s * dy;
    const double yLocal = -s * dx + c * dy;
    const double aa = a * a;
    const double bb = b * b;
    const double f = xLocal * xLocal / aa + yLocal * yLocal / bb - 1.0;
    const double gx = 2.0 * xLocal / aa;
    const double gy = 2.0 * yLocal / bb;
    return f / std::sqrt(gx * gx + gy * gy + eps);
}

bool fullEllipse(double spanAngleDeg)
{
    return std::abs(spanAngleDeg) >= 360.0;
}

double robustWeight(double residual, const EllipseRobustOptions& options)
{
    const double absResidual = std::abs(residual);
    switch (options.lossType) {
    case RobustLossType::None:
        return 1.0;
    case RobustLossType::Huber:
        if (options.huberDelta <= 0.0 || absResidual <= options.huberDelta) {
            return 1.0;
        }
        return options.huberDelta / std::max(absResidual, 1.0e-12);
    case RobustLossType::Tukey:
        if (options.tukeyC <= 0.0 || absResidual > options.tukeyC) {
            return 0.0;
        } else {
            const double t = residual / options.tukeyC;
            const double v = 1.0 - t * t;
            return v * v;
        }
    }
    return 1.0;
}

void updatePointDebug(FindEllipseResult* result, const EllipseRobustOptions& robust)
{
    if (!result) {
        return;
    }

    result->pointResiduals.clear();
    result->pointWeights.clear();
    result->pointResiduals.reserve(result->inputPoints.size());
    result->pointWeights.reserve(result->inputPoints.size());
    for (const EdgePoint& point : result->inputPoints) {
        const double residual = residualApproxGeometric(point, result->fittedEllipse);
        result->pointResiduals.push_back(residual);
        result->pointWeights.push_back(std::max(robust.minWeight, robustWeight(residual, robust)));
    }
}

void updateDiagnostics(FindEllipseResult* result, const FindEllipseParams& params, const QString& failureReason = {})
{
    if (!result) {
        return;
    }

    EllipseFitDiagnostics diagnostics;
    diagnostics.inputCount = static_cast<int>(result->inputPoints.size());
    diagnostics.candidateCount = diagnostics.inputCount;
    diagnostics.inlierCount = static_cast<int>(result->inlierPoints.size());
    diagnostics.outlierCount = static_cast<int>(result->outlierPoints.size());
    diagnostics.inlierRatio = diagnostics.inputCount > 0
        ? static_cast<double>(diagnostics.inlierCount) / static_cast<double>(diagnostics.inputCount)
        : 0.0;
    diagnostics.rmsError = result->rmsError;
    diagnostics.maxError = result->maxError;

    std::vector<double> absErrors;
    absErrors.reserve(result->inlierPoints.size());
    for (const EdgePoint& point : result->inlierPoints) {
        absErrors.push_back(std::abs(residualApproxGeometric(point, result->fittedEllipse)));
    }
    if (!absErrors.empty()) {
        diagnostics.meanAbsError = std::accumulate(absErrors.begin(), absErrors.end(), 0.0) / static_cast<double>(absErrors.size());
        std::sort(absErrors.begin(), absErrors.end());
        const size_t mid = absErrors.size() / 2;
        diagnostics.medianError = (absErrors.size() % 2) == 0
            ? 0.5 * (absErrors[mid - 1] + absErrors[mid])
            : absErrors[mid];
    }

    diagnostics.centerShiftFromExpected = std::hypot(result->fittedEllipse.centerX - result->expectedEllipse.centerX,
                                                     result->fittedEllipse.centerY - result->expectedEllipse.centerY);
    diagnostics.radiusAErrorRatio = std::abs(result->fittedEllipse.radiusA - result->expectedEllipse.radiusA)
        / std::max(1.0, result->expectedEllipse.radiusA);
    diagnostics.radiusBErrorRatio = std::abs(result->fittedEllipse.radiusB - result->expectedEllipse.radiusB)
        / std::max(1.0, result->expectedEllipse.radiusB);
    diagnostics.angleErrorDeg = angleDifference180(result->fittedEllipse.angleDeg, result->expectedEllipse.angleDeg);
    diagnostics.conditionHint = std::max(result->fittedEllipse.radiusA, result->fittedEllipse.radiusB)
        / std::max(1.0, std::min(result->fittedEllipse.radiusA, result->fittedEllipse.radiusB));
    diagnostics.failureReason = failureReason;
    if (diagnostics.failureReason.isEmpty() && !result->ok && !result->message.isEmpty()) {
        diagnostics.failureReason = result->message;
    }
    if (diagnostics.failureReason.isEmpty() && params.spanAngleDeg != 0.0) {
        diagnostics.failureReason.clear();
    }

    result->diagnostics = diagnostics;
}

} // namespace

FindEllipseTool::FindEllipseTool() = default;

void FindEllipseTool::setParams(const FindEllipseParams& params)
{
    m_params = params;
    m_params.radiusA = std::max(1.0, m_params.radiusA);
    m_params.radiusB = std::max(1.0, m_params.radiusB);
    if (m_params.radiusA < m_params.radiusB) {
        std::swap(m_params.radiusA, m_params.radiusB);
        m_params.angleDeg += 90.0;
    }
    m_params.angleDeg = normalizeAngle180(m_params.angleDeg);
    m_params.caliperCount = std::max(5, m_params.caliperCount);
    m_params.searchLength = std::max(1.0, m_params.searchLength);
    m_params.projectionWidth = std::max(0.0, m_params.projectionWidth);
    m_params.minResponse = std::max(0.0, m_params.minResponse);
    m_params.maxResidual = std::max(0.0, m_params.maxResidual);
    m_params.maxIterations = std::max(0, m_params.maxIterations);
    m_params.robust.huberDelta = std::max(0.0, m_params.robust.huberDelta);
    m_params.robust.tukeyC = std::max(0.0, m_params.robust.tukeyC);
    m_params.robust.irlsIterations = std::max(0, m_params.robust.irlsIterations);
    m_params.robust.minWeight = std::clamp(m_params.robust.minWeight, 0.0, 1.0);
}

const FindEllipseParams& FindEllipseTool::params() const
{
    return m_params;
}

FindEllipseResult FindEllipseTool::run(const ImageView& image) const
{
    FindEllipseResult result;
    result.expectedEllipse = normalizedExpected(m_params);
    result.fittedEllipse = result.expectedEllipse;

    if (!image.isValid()) {
        result.message = QStringLiteral("Invalid image");
        updateDiagnostics(&result, m_params, result.message);
        return result;
    }

    result.calipers = generateCalipers();
    result.inputPoints.reserve(result.calipers.size());
    result.caliperHitPoints.reserve(result.calipers.size());
    for (const CaliperRegion& caliper : result.calipers) {
        const EdgePoint point = runSingleCaliper(image, caliper);
        result.caliperHitPoints.push_back(point);
        if (point.valid) {
            result.inputPoints.push_back(point);
        }
    }

    if (result.inputPoints.size() < 5) {
        result.message = QStringLiteral("Not enough edge points");
        updateDiagnostics(&result, m_params, result.message);
        return result;
    }

    QString initialMessage;
    EllipseModel initial = buildInitialEllipse(result.inputPoints, initialMessage);
    splitInliersOutliers(result.inputPoints, initial, result.inlierPoints, result.outlierPoints);
    if (!m_params.enableRobustFiltering) {
        result.inlierPoints = result.inputPoints;
        result.outlierPoints.clear();
    }
    if (result.inlierPoints.size() < 5) {
        result.message = QStringLiteral("Not enough ellipse inliers after filtering");
        updateDiagnostics(&result, m_params, result.message);
        return result;
    }

    if (m_params.enableNonlinearRefine) {
        EllipseFitOptions options;
        options.maxResidual = m_params.maxResidual;
        options.minInlierCount = 5;
        options.maxIterations = m_params.maxIterations;
        options.robust = m_params.robust;
        EllipseFitter fitter;
        const EllipseFitResult fit = fitter.fit(result.inlierPoints, initial, options);
        if (!fit.ok) {
            result.fittedEllipse = fit.ellipse;
            result.message = QStringLiteral("Ellipse refinement failed: %1").arg(fit.message);
            updateDiagnostics(&result, m_params, result.message);
            return result;
        }
        result.fittedEllipse = fit.ellipse;

        splitInliersOutliers(result.inputPoints, result.fittedEllipse, result.inlierPoints, result.outlierPoints);
        if (result.inlierPoints.size() < 5) {
            result.message = QStringLiteral("Not enough ellipse inliers after refinement");
            updateDiagnostics(&result, m_params, result.message);
            return result;
        }

        const EllipseFitResult finalFit = fitter.fit(result.inlierPoints, result.fittedEllipse, options);
        if (finalFit.ok) {
            result.fittedEllipse = finalFit.ellipse;
        }
    } else {
        result.fittedEllipse = initial;
    }

    double sumSq = 0.0;
    result.maxError = 0.0;
    for (const EdgePoint& point : result.inlierPoints) {
        const double error = std::abs(residualApproxGeometric(point, result.fittedEllipse));
        sumSq += error * error;
        result.maxError = std::max(result.maxError, error);
    }
    result.rmsError = result.inlierPoints.empty() ? 0.0 : std::sqrt(sumSq / static_cast<double>(result.inlierPoints.size()));
    updatePointDebug(&result, m_params.robust);

    const double expectedRadius = std::max(result.expectedEllipse.radiusA, result.expectedEllipse.radiusB);
    const double centerShift = std::hypot(result.fittedEllipse.centerX - result.expectedEllipse.centerX,
                                          result.fittedEllipse.centerY - result.expectedEllipse.centerY);
    const double radiusARatio = result.fittedEllipse.radiusA / std::max(1.0, result.expectedEllipse.radiusA);
    const double radiusBRatio = result.fittedEllipse.radiusB / std::max(1.0, result.expectedEllipse.radiusB);
    if (centerShift > expectedRadius * 0.5) {
        result.message = QStringLiteral("Fitted ellipse center is too far from expected ellipse");
        updateDiagnostics(&result, m_params, result.message);
        return result;
    }
    if (radiusARatio < 0.5 || radiusARatio > 1.5 || radiusBRatio < 0.5 || radiusBRatio > 1.5) {
        result.message = QStringLiteral("Fitted ellipse radii differ too much from expected ellipse");
        updateDiagnostics(&result, m_params, result.message);
        return result;
    }
    if (result.rmsError > std::max(1.0, m_params.maxResidual) * 2.0) {
        result.message = QStringLiteral("Ellipse RMS residual is too high");
        updateDiagnostics(&result, m_params, result.message);
        return result;
    }

    result.score = computeScore(result);
    result.ok = true;
    const double angleError = angleDifference180(result.fittedEllipse.angleDeg, result.expectedEllipse.angleDeg);
    result.message = angleError > 20.0
        ? QStringLiteral("OK; fitted angle differs from expected angle")
        : QStringLiteral("OK");
    updateDiagnostics(&result, m_params);
    return result;
}

std::vector<CaliperRegion> FindEllipseTool::generateCalipers() const
{
    std::vector<CaliperRegion> calipers;
    const EllipseModel ellipse = normalizedExpected(m_params);
    const int count = std::max(5, m_params.caliperCount);
    calipers.reserve(static_cast<size_t>(count));

    const double theta = degToRad(ellipse.angleDeg);
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    const double denominator = fullEllipse(m_params.spanAngleDeg)
        ? static_cast<double>(count)
        : static_cast<double>(std::max(1, count - 1));
    const double step = m_params.spanAngleDeg / denominator;

    for (int i = 0; i < count; ++i) {
        const double tDeg = m_params.startAngleDeg + static_cast<double>(i) * step;
        const double t = degToRad(tDeg);
        const double xLocal = ellipse.radiusA * std::cos(t);
        const double yLocal = ellipse.radiusB * std::sin(t);
        double nxLocal = xLocal / (ellipse.radiusA * ellipse.radiusA);
        double nyLocal = yLocal / (ellipse.radiusB * ellipse.radiusB);
        const double normalLength = std::hypot(nxLocal, nyLocal);
        if (normalLength <= 1.0e-12) {
            continue;
        }
        nxLocal /= normalLength;
        nyLocal /= normalLength;

        double nx = c * nxLocal - s * nyLocal;
        double ny = s * nxLocal + c * nyLocal;
        if (m_params.searchDirection == EllipseSearchDirection::FromOutsideToInside) {
            nx = -nx;
            ny = -ny;
        }

        CaliperRegion caliper;
        caliper.centerX = ellipse.centerX + c * xLocal - s * yLocal;
        caliper.centerY = ellipse.centerY + s * xLocal + c * yLocal;
        caliper.length = m_params.searchLength;
        caliper.width = m_params.projectionWidth;
        caliper.angleDeg = radToDeg(std::atan2(ny, nx));
        calipers.push_back(caliper);
    }

    return calipers;
}

EdgePoint FindEllipseTool::runSingleCaliper(const ImageView& image, const CaliperRegion& caliper) const
{
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
    const CaliperResult result = caliperTool.run(image, caliper);
    if (result.ok && !result.selectedEdges.empty()) {
        const EdgePoint& point = result.selectedEdges.front();
        if (point.valid && std::abs(point.response) >= m_params.minResponse) {
            return point;
        }
    }
    return {};
}

EllipseModel FindEllipseTool::buildInitialEllipse(const std::vector<EdgePoint>&, QString& message) const
{
    message = QStringLiteral("Using expected ellipse as initial estimate");
    return normalizedExpected(m_params);
}

void FindEllipseTool::splitInliersOutliers(const std::vector<EdgePoint>& points,
                                           const EllipseModel& ellipse,
                                           std::vector<EdgePoint>& inliers,
                                           std::vector<EdgePoint>& outliers) const
{
    inliers.clear();
    outliers.clear();
    for (const EdgePoint& point : points) {
        const double error = std::abs(residualApproxGeometric(point, ellipse));
        if (error <= m_params.maxResidual) {
            inliers.push_back(point);
        } else {
            outliers.push_back(point);
        }
    }
}

double FindEllipseTool::computeScore(const FindEllipseResult& result) const
{
    const double inlierRatio = result.inputPoints.empty()
        ? 0.0
        : static_cast<double>(result.inlierPoints.size()) / static_cast<double>(result.inputPoints.size());
    const double errorScore = std::exp(-result.rmsError / std::max(1.0e-6, m_params.maxResidual));
    return std::clamp(inlierRatio * errorScore, 0.0, 1.0);
}

} // namespace VisionTools
