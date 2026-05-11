#include "VisionTools/EllipseFitter.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>

namespace VisionTools {

namespace {

constexpr double pi = 3.14159265358979323846;
constexpr int parameterCount = 5;

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

bool isFiniteEllipse(const EllipseModel& ellipse)
{
    return std::isfinite(ellipse.centerX)
        && std::isfinite(ellipse.centerY)
        && std::isfinite(ellipse.radiusA)
        && std::isfinite(ellipse.radiusB)
        && std::isfinite(ellipse.angleDeg);
}

double squaredNorm(const std::array<double, parameterCount>& values)
{
    double sum = 0.0;
    for (double value : values) {
        sum += value * value;
    }
    return sum;
}

double residualForModel(const EdgePoint& point, const EllipseModel& ellipse)
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

std::vector<double> computeWeights(const std::vector<EdgePoint>& points,
                                   const EllipseModel& ellipse,
                                   const EllipseRobustOptions& options)
{
    std::vector<double> weights;
    weights.reserve(points.size());
    const double minWeight = std::max(0.0, options.minWeight);
    for (const EdgePoint& point : points) {
        const double residual = residualForModel(point, ellipse);
        weights.push_back(std::max(minWeight, robustWeight(residual, options)));
    }
    return weights;
}

double costForModel(const std::vector<EdgePoint>& points,
                    const EllipseModel& ellipse,
                    const std::vector<double>& weights)
{
    if (points.empty()) {
        return std::numeric_limits<double>::max();
    }

    double sumSq = 0.0;
    for (size_t i = 0; i < points.size(); ++i) {
        const double residual = residualForModel(points[i], ellipse);
        const double weight = i < weights.size() ? weights[i] : 1.0;
        sumSq += weight * residual * residual;
    }
    return 0.5 * sumSq;
}

EllipseModel addDelta(const EllipseModel& ellipse, const std::array<double, parameterCount>& delta)
{
    EllipseModel next = ellipse;
    next.centerX += delta[0];
    next.centerY += delta[1];
    next.radiusA += delta[2];
    next.radiusB += delta[3];
    next.angleDeg += radToDeg(delta[4]);
    return next;
}

bool solveLinearSystem(std::array<std::array<double, parameterCount>, parameterCount> matrix,
                       std::array<double, parameterCount> rhs,
                       std::array<double, parameterCount>* solution)
{
    if (!solution) {
        return false;
    }

    for (int i = 0; i < parameterCount; ++i) {
        int pivot = i;
        double pivotAbs = std::abs(matrix[i][i]);
        for (int row = i + 1; row < parameterCount; ++row) {
            const double candidate = std::abs(matrix[row][i]);
            if (candidate > pivotAbs) {
                pivot = row;
                pivotAbs = candidate;
            }
        }

        if (pivotAbs <= 1.0e-12) {
            return false;
        }

        if (pivot != i) {
            std::swap(matrix[pivot], matrix[i]);
            std::swap(rhs[pivot], rhs[i]);
        }

        const double divisor = matrix[i][i];
        for (int col = i; col < parameterCount; ++col) {
            matrix[i][col] /= divisor;
        }
        rhs[i] /= divisor;

        for (int row = 0; row < parameterCount; ++row) {
            if (row == i) {
                continue;
            }
            const double factor = matrix[row][i];
            for (int col = i; col < parameterCount; ++col) {
                matrix[row][col] -= factor * matrix[i][col];
            }
            rhs[row] -= factor * rhs[i];
        }
    }

    *solution = rhs;
    return true;
}

} // namespace

EllipseFitter::EllipseFitter() = default;

EllipseFitResult EllipseFitter::fit(const std::vector<EdgePoint>& points,
                                    const EllipseModel& initial,
                                    const EllipseFitOptions& options) const
{
    EllipseFitResult result;
    result.inputPoints = points;
    result.inlierPoints = points;
    result.ellipse = initial;

    EllipseFitOptions normalizedOptions = options;
    normalizedOptions.maxResidual = std::max(0.0, normalizedOptions.maxResidual);
    normalizedOptions.minInlierCount = std::max(5, normalizedOptions.minInlierCount);
    normalizedOptions.maxIterations = std::max(0, normalizedOptions.maxIterations);
    normalizedOptions.initialLambda = std::max(1.0e-9, normalizedOptions.initialLambda);
    normalizedOptions.minRadius = std::max(1.0, normalizedOptions.minRadius);
    normalizedOptions.robust.huberDelta = std::max(0.0, normalizedOptions.robust.huberDelta);
    normalizedOptions.robust.tukeyC = std::max(0.0, normalizedOptions.robust.tukeyC);
    normalizedOptions.robust.irlsIterations = std::max(0, normalizedOptions.robust.irlsIterations);
    normalizedOptions.robust.minWeight = std::clamp(normalizedOptions.robust.minWeight, 0.0, 1.0);

    if (points.size() < static_cast<size_t>(normalizedOptions.minInlierCount)) {
        result.message = QStringLiteral("Not enough input points");
        return result;
    }
    if (!normalizeEllipse(result.ellipse, normalizedOptions.minRadius)) {
        result.message = QStringLiteral("Invalid initial ellipse");
        return result;
    }

    if (!refineLevenbergMarquardt(points, result.ellipse, normalizedOptions, result)) {
        updateErrorMetrics(&result);
        return result;
    }

    result.ok = true;
    result.message = QStringLiteral("OK");
    updateErrorMetrics(&result);
    return result;
}

double EllipseFitter::residualApproxGeometric(const EdgePoint& point, const EllipseModel& ellipse) const
{
    return residualForModel(point, ellipse);
}

bool EllipseFitter::refineLevenbergMarquardt(const std::vector<EdgePoint>& points,
                                             EllipseModel& ellipse,
                                             const EllipseFitOptions& options,
                                             EllipseFitResult& result) const
{
    double lambda = options.initialLambda;
    std::vector<double> weights = computeWeights(points, ellipse, options.robust);
    double currentCost = costForModel(points, ellipse, weights);
    if (!std::isfinite(currentCost)) {
        result.message = QStringLiteral("Initial ellipse cost is invalid");
        return false;
    }

    for (int iter = 0; iter < options.maxIterations; ++iter) {
        if (iter < options.robust.irlsIterations || weights.size() != points.size()) {
            weights = computeWeights(points, ellipse, options.robust);
            currentCost = costForModel(points, ellipse, weights);
        }

        std::array<std::array<double, parameterCount>, parameterCount> normal {};
        std::array<double, parameterCount> gradient {};

        for (size_t pointIndex = 0; pointIndex < points.size(); ++pointIndex) {
            const EdgePoint& point = points[pointIndex];
            const double residual = residualForModel(point, ellipse);
            const double weight = pointIndex < weights.size() ? weights[pointIndex] : 1.0;
            std::array<double, parameterCount> jacobian {};

            const double steps[parameterCount] = {
                1.0e-3 * std::max(1.0, std::abs(ellipse.centerX)),
                1.0e-3 * std::max(1.0, std::abs(ellipse.centerY)),
                1.0e-4 * std::max(1.0, std::abs(ellipse.radiusA)),
                1.0e-4 * std::max(1.0, std::abs(ellipse.radiusB)),
                1.0e-5
            };

            for (int p = 0; p < parameterCount; ++p) {
                std::array<double, parameterCount> delta {};
                delta[p] = steps[p];
                EllipseModel plus = addDelta(ellipse, delta);
                if (!normalizeEllipse(plus, options.minRadius)) {
                    plus = ellipse;
                }
                delta[p] = -steps[p];
                EllipseModel minus = addDelta(ellipse, delta);
                if (!normalizeEllipse(minus, options.minRadius)) {
                    minus = ellipse;
                }
                const double plusResidual = residualForModel(point, plus);
                const double minusResidual = residualForModel(point, minus);
                jacobian[p] = (plusResidual - minusResidual) / (2.0 * steps[p]);
            }

            for (int row = 0; row < parameterCount; ++row) {
                gradient[row] += weight * jacobian[row] * residual;
                for (int col = 0; col < parameterCount; ++col) {
                    normal[row][col] += weight * jacobian[row] * jacobian[col];
                }
            }
        }

        for (int i = 0; i < parameterCount; ++i) {
            normal[i][i] += lambda;
            gradient[i] = -gradient[i];
        }

        std::array<double, parameterCount> delta {};
        if (!solveLinearSystem(normal, gradient, &delta)) {
            lambda *= 10.0;
            continue;
        }

        if (squaredNorm(delta) <= 1.0e-12) {
            break;
        }

        EllipseModel candidate = addDelta(ellipse, delta);
        if (!normalizeEllipse(candidate, options.minRadius)) {
            lambda *= 10.0;
            continue;
        }

        const std::vector<double> candidateWeights = (iter < options.robust.irlsIterations)
            ? computeWeights(points, candidate, options.robust)
            : weights;
        const double candidateCost = costForModel(points, candidate, candidateWeights);
        if (std::isfinite(candidateCost) && candidateCost < currentCost) {
            const double costDelta = currentCost - candidateCost;
            ellipse = candidate;
            weights = candidateWeights;
            currentCost = candidateCost;
            lambda = std::max(lambda * 0.35, 1.0e-9);
            if (costDelta <= 1.0e-9 * std::max(1.0, currentCost)) {
                break;
            }
        } else {
            lambda *= 10.0;
        }
    }

    result.ellipse = ellipse;
    result.pointWeights = computeWeights(points, ellipse, options.robust);
    if (!isFiniteEllipse(result.ellipse)) {
        result.message = QStringLiteral("Refined ellipse contains invalid values");
        return false;
    }
    return true;
}

bool EllipseFitter::normalizeEllipse(EllipseModel& ellipse, double minRadius) const
{
    if (!isFiniteEllipse(ellipse)) {
        return false;
    }

    ellipse.radiusA = std::max(minRadius, ellipse.radiusA);
    ellipse.radiusB = std::max(minRadius, ellipse.radiusB);
    if (ellipse.radiusA < ellipse.radiusB) {
        std::swap(ellipse.radiusA, ellipse.radiusB);
        ellipse.angleDeg += 90.0;
    }
    ellipse.angleDeg = normalizeAngle180(ellipse.angleDeg);
    return true;
}

void EllipseFitter::updateErrorMetrics(EllipseFitResult* result) const
{
    if (!result || result->inlierPoints.empty()) {
        return;
    }

    double sumSq = 0.0;
    double maxError = 0.0;
    result->pointResiduals.clear();
    if (result->pointWeights.size() != result->inputPoints.size()) {
        result->pointWeights.assign(result->inputPoints.size(), 1.0);
    }
    for (const EdgePoint& point : result->inputPoints) {
        result->pointResiduals.push_back(residualApproxGeometric(point, result->ellipse));
    }

    for (const EdgePoint& point : result->inlierPoints) {
        const double error = std::abs(residualApproxGeometric(point, result->ellipse));
        sumSq += error * error;
        maxError = std::max(maxError, error);
    }

    result->rmsError = std::sqrt(sumSq / static_cast<double>(result->inlierPoints.size()));
    result->maxError = maxError;
    result->score = result->inputPoints.empty()
        ? 0.0
        : static_cast<double>(result->inlierPoints.size()) / static_cast<double>(result->inputPoints.size());
}

} // namespace VisionTools
