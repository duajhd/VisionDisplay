#include "VisionTools/CircleFitter.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace VisionTools {

namespace {

double pointWeight(const EdgePoint& point)
{
    return std::max(1.0e-6, std::abs(point.response));
}

double huberWeight(double residual, double delta)
{
    const double absResidual = std::abs(residual);
    if (delta <= 0.0 || absResidual <= delta) {
        return 1.0;
    }
    return delta / absResidual;
}

bool solve3x3(double a[3][3], double b[3], double x[3])
{
    for (int col = 0; col < 3; ++col) {
        int pivot = col;
        double pivotAbs = std::abs(a[col][col]);
        for (int row = col + 1; row < 3; ++row) {
            const double value = std::abs(a[row][col]);
            if (value > pivotAbs) {
                pivot = row;
                pivotAbs = value;
            }
        }
        if (pivotAbs <= std::numeric_limits<double>::epsilon()) {
            return false;
        }
        if (pivot != col) {
            for (int k = col; k < 3; ++k) {
                std::swap(a[col][k], a[pivot][k]);
            }
            std::swap(b[col], b[pivot]);
        }

        const double diag = a[col][col];
        for (int k = col; k < 3; ++k) {
            a[col][k] /= diag;
        }
        b[col] /= diag;

        for (int row = 0; row < 3; ++row) {
            if (row == col) {
                continue;
            }
            const double factor = a[row][col];
            for (int k = col; k < 3; ++k) {
                a[row][k] -= factor * a[col][k];
            }
            b[row] -= factor * b[col];
        }
    }

    x[0] = b[0];
    x[1] = b[1];
    x[2] = b[2];
    return true;
}

} // namespace

CircleFitter::CircleFitter() = default;

void CircleFitter::setParams(const CircleFitParams& params)
{
    m_params = params;
    m_params.maxResidual = std::max(0.0, m_params.maxResidual);
    m_params.minInlierCount = std::max(3, m_params.minInlierCount);
    m_params.maxIterations = std::max(0, m_params.maxIterations);
    m_params.damping = std::max(0.0, m_params.damping);
    m_params.ransacIterations = std::max(0, m_params.ransacIterations);
    m_params.ransacResidual = std::max(0.0, m_params.ransacResidual);
    m_params.huberDelta = std::max(0.0, m_params.huberDelta);
}

const CircleFitParams& CircleFitter::params() const
{
    return m_params;
}

CircleFitResult CircleFitter::fit(const std::vector<EdgePoint>& points) const
{
    CircleFitResult result;
    result.inputPoints = points;

    if (points.size() < static_cast<size_t>(std::max(3, m_params.minInlierCount))) {
        result.message = QStringLiteral("Not enough input points");
        return result;
    }

    const CircleModel initialCircle = m_params.enableRansac ? fitRansac(points) : fitAlgebraic(points);
    if (initialCircle.radius <= 0.0) {
        result.message = QStringLiteral("Initial circle fit failed");
        return result;
    }

    for (const EdgePoint& point : points) {
        const double distance = std::abs(residual(initialCircle, point));
        if (distance <= m_params.maxResidual) {
            result.inlierPoints.push_back(point);
        } else {
            result.outlierPoints.push_back(point);
        }
    }

    if (result.inlierPoints.size() < static_cast<size_t>(m_params.minInlierCount)) {
        result.message = QStringLiteral("Not enough inliers after residual filtering");
        result.circle = initialCircle;
        updateErrorMetrics(&result);
        return result;
    }

    result.circle = optimizeGeometric(fitAlgebraic(result.inlierPoints), result.inlierPoints);
    if (result.circle.radius <= 0.0) {
        result.message = QStringLiteral("Geometric circle optimization failed");
        return result;
    }

    updateErrorMetrics(&result);
    result.ok = true;
    result.score = points.empty() ? 0.0 : static_cast<double>(result.inlierPoints.size()) / static_cast<double>(points.size());
    result.message = QStringLiteral("OK");
    return result;
}

CircleModel CircleFitter::fitAlgebraic(const std::vector<EdgePoint>& points) const
{
    CircleModel circle;
    if (points.size() < 3) {
        return circle;
    }

    double normal[3][3] = {};
    double rhs[3] = {};

    for (const EdgePoint& point : points) {
        const double w = pointWeight(point);
        const double row[3] = {point.x, point.y, 1.0};
        const double target = -(point.x * point.x + point.y * point.y);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                normal[r][c] += w * row[r] * row[c];
            }
            rhs[r] += w * row[r] * target;
        }
    }

    double solution[3] = {};
    if (!solve3x3(normal, rhs, solution)) {
        return circle;
    }

    const double a = solution[0];
    const double b = solution[1];
    const double c = solution[2];
    const double radiusSq = (a * a + b * b) * 0.25 - c;
    if (radiusSq <= 0.0) {
        return circle;
    }

    circle.centerX = -a * 0.5;
    circle.centerY = -b * 0.5;
    circle.radius = std::sqrt(radiusSq);
    return circle;
}

CircleModel CircleFitter::fitRansac(const std::vector<EdgePoint>& points) const
{
    if (points.size() < 3 || m_params.ransacIterations <= 0) {
        return fitAlgebraic(points);
    }

    std::mt19937 rng(2027);
    std::uniform_int_distribution<size_t> pick(0, points.size() - 1);
    const double threshold = m_params.ransacResidual > 0.0 ? m_params.ransacResidual : m_params.maxResidual;
    size_t bestCount = 0;
    double bestRms = std::numeric_limits<double>::max();
    std::vector<EdgePoint> bestInliers;

    for (int iter = 0; iter < m_params.ransacIterations; ++iter) {
        const size_t ia = pick(rng);
        const size_t ib = pick(rng);
        const size_t ic = pick(rng);
        if (ia == ib || ia == ic || ib == ic) {
            continue;
        }

        const CircleModel candidate = circleFromThreePoints(points[ia], points[ib], points[ic]);
        if (candidate.radius <= 0.0) {
            continue;
        }

        std::vector<EdgePoint> inliers;
        double sumSq = 0.0;
        for (const EdgePoint& point : points) {
            const double error = std::abs(residual(candidate, point));
            if (error <= threshold) {
                inliers.push_back(point);
                sumSq += error * error;
            }
        }

        const double rms = inliers.empty() ? std::numeric_limits<double>::max()
                                           : std::sqrt(sumSq / static_cast<double>(inliers.size()));
        if (inliers.size() > bestCount || (inliers.size() == bestCount && rms < bestRms)) {
            bestCount = inliers.size();
            bestRms = rms;
            bestInliers = std::move(inliers);
        }
    }

    if (bestInliers.size() < static_cast<size_t>(m_params.minInlierCount)) {
        return fitAlgebraic(points);
    }
    return fitAlgebraic(bestInliers);
}

CircleModel CircleFitter::circleFromThreePoints(const EdgePoint& a, const EdgePoint& b, const EdgePoint& c) const
{
    CircleModel circle;
    const double d = 2.0 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
    if (std::abs(d) <= std::numeric_limits<double>::epsilon()) {
        return circle;
    }

    const double a2 = a.x * a.x + a.y * a.y;
    const double b2 = b.x * b.x + b.y * b.y;
    const double c2 = c.x * c.x + c.y * c.y;
    circle.centerX = (a2 * (b.y - c.y) + b2 * (c.y - a.y) + c2 * (a.y - b.y)) / d;
    circle.centerY = (a2 * (c.x - b.x) + b2 * (a.x - c.x) + c2 * (b.x - a.x)) / d;
    const double dx = a.x - circle.centerX;
    const double dy = a.y - circle.centerY;
    circle.radius = std::sqrt(dx * dx + dy * dy);
    return circle;
}

CircleModel CircleFitter::optimizeGeometric(const CircleModel& initialCircle,
                                            const std::vector<EdgePoint>& points) const
{
    CircleModel circle = initialCircle;
    if (circle.radius <= 0.0 || points.size() < 3) {
        return circle;
    }

    for (int iter = 0; iter < m_params.maxIterations; ++iter) {
        double normal[3][3] = {};
        double rhs[3] = {};

        for (const EdgePoint& point : points) {
            const double dx = point.x - circle.centerX;
            const double dy = point.y - circle.centerY;
            const double distance = std::sqrt(dx * dx + dy * dy);
            if (distance <= std::numeric_limits<double>::epsilon()) {
                continue;
            }

            const double res = distance - circle.radius;
            const double jac[3] = {-dx / distance, -dy / distance, -1.0};
            const double robustWeight = m_params.enableHuber ? huberWeight(res, m_params.huberDelta) : 1.0;
            const double w = pointWeight(point) * robustWeight;
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    normal[r][c] += w * jac[r] * jac[c];
                }
                rhs[r] += -w * jac[r] * res;
            }
        }

        for (int i = 0; i < 3; ++i) {
            normal[i][i] += m_params.damping;
        }

        double delta[3] = {};
        if (!solve3x3(normal, rhs, delta)) {
            break;
        }

        circle.centerX += delta[0];
        circle.centerY += delta[1];
        circle.radius += delta[2];
        if (circle.radius <= 0.0) {
            circle.radius = initialCircle.radius;
            break;
        }

        const double stepNorm = std::sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
        if (stepNorm < 1.0e-6) {
            break;
        }
    }

    return circle;
}

double CircleFitter::residual(const CircleModel& circle, const EdgePoint& point) const
{
    const double dx = point.x - circle.centerX;
    const double dy = point.y - circle.centerY;
    return std::sqrt(dx * dx + dy * dy) - circle.radius;
}

void CircleFitter::updateErrorMetrics(CircleFitResult* result) const
{
    if (!result || result->inlierPoints.empty()) {
        return;
    }

    double sumSq = 0.0;
    double maxError = 0.0;
    for (const EdgePoint& point : result->inlierPoints) {
        const double error = std::abs(residual(result->circle, point));
        sumSq += error * error;
        maxError = std::max(maxError, error);
    }

    result->rmsError = std::sqrt(sumSq / static_cast<double>(result->inlierPoints.size()));
    result->maxError = maxError;
}

} // namespace VisionTools
