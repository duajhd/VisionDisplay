#include "VisionTools/LineFitter.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace VisionTools {

namespace {

constexpr double pi = 3.14159265358979323846;

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

} // namespace

LineFitter::LineFitter() = default;

void LineFitter::setParams(const LineFitParams& params)
{
    m_params = params;
    m_params.maxResidual = std::max(0.0, m_params.maxResidual);
    m_params.minInlierCount = std::max(2, m_params.minInlierCount);
    m_params.ransacIterations = std::max(0, m_params.ransacIterations);
    m_params.ransacResidual = std::max(0.0, m_params.ransacResidual);
    m_params.huberDelta = std::max(0.0, m_params.huberDelta);
    m_params.robustIterations = std::max(0, m_params.robustIterations);
}

const LineFitParams& LineFitter::params() const
{
    return m_params;
}

LineFitResult LineFitter::fit(const std::vector<EdgePoint>& points) const
{
    LineFitResult result;
    result.inputPoints = points;

    if (points.size() < static_cast<size_t>(std::max(2, m_params.minInlierCount))) {
        result.message = QStringLiteral("Not enough input points");
        return result;
    }

    const LineModel initialLine = m_params.enableRansac ? fitRansac(points) : fitWeightedTls(points);
    for (const EdgePoint& point : points) {
        const double distance = std::abs(residual(initialLine, point));
        if (distance <= m_params.maxResidual) {
            result.inlierPoints.push_back(point);
        } else {
            result.outlierPoints.push_back(point);
        }
    }

    if (result.inlierPoints.size() < static_cast<size_t>(m_params.minInlierCount)) {
        result.message = QStringLiteral("Not enough inliers after residual filtering");
        result.line = initialLine;
        updateErrorMetrics(&result);
        return result;
    }

    result.line = robustRefit(fitWeightedTls(result.inlierPoints), result.inlierPoints);
    updateErrorMetrics(&result);
    result.ok = true;
    result.score = points.empty() ? 0.0 : static_cast<double>(result.inlierPoints.size()) / static_cast<double>(points.size());
    result.message = QStringLiteral("OK");
    return result;
}

LineModel LineFitter::fitWeightedTls(const std::vector<EdgePoint>& points) const
{
    return fitWeightedTls(points, {});
}

LineModel LineFitter::fitWeightedTls(const std::vector<EdgePoint>& points,
                                     const std::vector<double>& robustWeights) const
{
    LineModel line;
    if (points.size() < 2) {
        return line;
    }

    double weightSum = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    for (size_t i = 0; i < points.size(); ++i) {
        const EdgePoint& point = points[i];
        const double rw = i < robustWeights.size() ? robustWeights[i] : 1.0;
        const double w = pointWeight(point) * std::max(0.0, rw);
        weightSum += w;
        cx += point.x * w;
        cy += point.y * w;
    }
    if (weightSum <= std::numeric_limits<double>::epsilon()) {
        return line;
    }
    cx /= weightSum;
    cy /= weightSum;

    double sxx = 0.0;
    double sxy = 0.0;
    double syy = 0.0;
    for (size_t i = 0; i < points.size(); ++i) {
        const EdgePoint& point = points[i];
        const double rw = i < robustWeights.size() ? robustWeights[i] : 1.0;
        const double w = pointWeight(point) * std::max(0.0, rw);
        const double dx = point.x - cx;
        const double dy = point.y - cy;
        sxx += w * dx * dx;
        sxy += w * dx * dy;
        syy += w * dy * dy;
    }

    const double directionAngle = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
    const double dirX = std::cos(directionAngle);
    const double dirY = std::sin(directionAngle);
    double nx = -dirY;
    double ny = dirX;
    double rho = nx * cx + ny * cy;
    if (rho < 0.0) {
        rho = -rho;
        nx = -nx;
        ny = -ny;
    }

    double minT = std::numeric_limits<double>::max();
    double maxT = -std::numeric_limits<double>::max();
    for (const EdgePoint& point : points) {
        const double t = (point.x - cx) * dirX + (point.y - cy) * dirY;
        minT = std::min(minT, t);
        maxT = std::max(maxT, t);
    }

    line.theta = std::atan2(ny, nx);
    if (line.theta < 0.0) {
        line.theta += 2.0 * pi;
    }
    line.rho = rho;
    line.x1 = cx + minT * dirX;
    line.y1 = cy + minT * dirY;
    line.x2 = cx + maxT * dirX;
    line.y2 = cy + maxT * dirY;
    line.nx = nx;
    line.ny = ny;
    return line;
}

LineModel LineFitter::fitRansac(const std::vector<EdgePoint>& points) const
{
    if (points.size() < 2 || m_params.ransacIterations <= 0) {
        return fitWeightedTls(points);
    }

    std::mt19937 rng(1337);
    std::uniform_int_distribution<size_t> pick(0, points.size() - 1);
    const double threshold = m_params.ransacResidual > 0.0 ? m_params.ransacResidual : m_params.maxResidual;
    size_t bestCount = 0;
    double bestRms = std::numeric_limits<double>::max();
    std::vector<EdgePoint> bestInliers;

    for (int iter = 0; iter < m_params.ransacIterations; ++iter) {
        size_t ia = pick(rng);
        size_t ib = pick(rng);
        if (ia == ib) {
            continue;
        }

        const LineModel candidate = lineFromTwoPoints(points[ia], points[ib]);
        if (qFuzzyIsNull(candidate.nx) && qFuzzyIsNull(candidate.ny)) {
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
        return fitWeightedTls(points);
    }
    return fitWeightedTls(bestInliers);
}

LineModel LineFitter::lineFromTwoPoints(const EdgePoint& a, const EdgePoint& b) const
{
    LineModel line;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length = std::sqrt(dx * dx + dy * dy);
    if (length <= std::numeric_limits<double>::epsilon()) {
        return line;
    }

    const double dirX = dx / length;
    const double dirY = dy / length;
    double nx = -dirY;
    double ny = dirX;
    double rho = nx * a.x + ny * a.y;
    if (rho < 0.0) {
        rho = -rho;
        nx = -nx;
        ny = -ny;
    }

    line.theta = std::atan2(ny, nx);
    if (line.theta < 0.0) {
        line.theta += 2.0 * pi;
    }
    line.rho = rho;
    line.x1 = a.x;
    line.y1 = a.y;
    line.x2 = b.x;
    line.y2 = b.y;
    line.nx = nx;
    line.ny = ny;
    return line;
}

LineModel LineFitter::robustRefit(const LineModel& initialLine, const std::vector<EdgePoint>& points) const
{
    LineModel line = initialLine;
    if (!m_params.enableHuber || points.size() < 2 || m_params.robustIterations <= 0) {
        return line;
    }

    std::vector<double> weights(points.size(), 1.0);
    for (int iter = 0; iter < m_params.robustIterations; ++iter) {
        for (size_t i = 0; i < points.size(); ++i) {
            weights[i] = huberWeight(residual(line, points[i]), m_params.huberDelta);
        }
        line = fitWeightedTls(points, weights);
    }
    return line;
}

double LineFitter::residual(const LineModel& line, const EdgePoint& point) const
{
    return line.nx * point.x + line.ny * point.y - line.rho;
}

void LineFitter::updateErrorMetrics(LineFitResult* result) const
{
    if (!result || result->inlierPoints.empty()) {
        return;
    }

    double sumSq = 0.0;
    double maxError = 0.0;
    for (const EdgePoint& point : result->inlierPoints) {
        const double error = std::abs(residual(result->line, point));
        sumSq += error * error;
        maxError = std::max(maxError, error);
    }

    result->rmsError = std::sqrt(sumSq / static_cast<double>(result->inlierPoints.size()));
    result->maxError = maxError;
}

} // namespace VisionTools
