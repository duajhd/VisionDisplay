#include "shape_match/pipeline_v3/PoseRefinerV3.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>

namespace ShapeMatch {
namespace {

cv::Mat gray8(const cv::Mat& image)
{
    cv::Mat gray;
    if (image.channels() == 1) gray = image;
    else if (image.channels() == 3) cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    else cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
    if (gray.depth() == CV_8U) return gray;
    cv::Mat out; gray.convertTo(out, CV_8U); return out;
}

float bilinear(const cv::Mat& image, float x, float y)
{
    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    if (x0 < 0 || y0 < 0 || x0 + 1 >= image.cols || y0 + 1 >= image.rows) return 0.0f;
    const float fx = x - x0, fy = y - y0;
    const float* r0 = image.ptr<float>(y0);
    const float* r1 = image.ptr<float>(y0 + 1);
    return (1.0f - fy) * ((1.0f - fx) * r0[x0] + fx * r0[x0 + 1])
         + fy * ((1.0f - fx) * r1[x0] + fx * r1[x0 + 1]);
}

FineEdgeMapV3 buildEdges(const cv::Mat& image)
{
    cv::Mat blurred;
    cv::GaussianBlur(gray8(image), blurred, cv::Size(3, 3), 0.8, 0.8, cv::BORDER_REPLICATE);
    FineEdgeMapV3 out;
    cv::Scharr(blurred, out.gx, CV_32F, 1, 0, 1.0 / 16.0);
    cv::Scharr(blurred, out.gy, CV_32F, 0, 1, 1.0 / 16.0);
    cv::magnitude(out.gx, out.gy, out.magnitude);
    return out;
}

} // namespace

FineEdgeMapV3 PoseRefinerV3::buildEdgeMap(const cv::Mat& image) const
{
    return image.empty() ? FineEdgeMapV3{} : buildEdges(image);
}

MatchResultV3 PoseRefinerV3::refine(const cv::Mat& image, const ShapeModelV3& model,
                                    const MatchResultV3& initial, const ShapeSearchParametersV3& p) const
{
    return refine(buildEdgeMap(image), model, initial, p);
}

MatchResultV3 PoseRefinerV3::refine(const FineEdgeMapV3& edges, const ShapeModelV3& model,
                                    const MatchResultV3& initial,
                                    const ShapeSearchParametersV3& p) const
{
    MatchResultV3 out = initial;
    if (edges.magnitude.empty() || model.points.empty() || p.subpixelMaxIterations <= 0) return out;
    double tx = initial.pose.x, ty = initial.pose.y, theta = initial.pose.theta;
    const double radius = std::max(1.0f, p.subpixelSearchRadius);
    const double cosTolerance = std::cos(std::clamp(p.subpixelOrientationToleranceRadians,
                                                    0.0f, static_cast<float>(kPi * 0.5)));
    const int minRequired = std::max(6, p.subpixelMinCorrespondences);

    for (int iteration = 0; iteration < p.subpixelMaxIterations; ++iteration) {
        cv::Matx33d h = cv::Matx33d::zeros();
        cv::Vec3d g(0.0, 0.0, 0.0);
        double weightedResidual2 = 0.0, weightSum = 0.0;
        int valid = 0;
        const double c = std::cos(theta), s = std::sin(theta);
        for (const ShapePointV3& point : model.points) {
            const double rx = c * point.x - s * point.y;
            const double ry = s * point.x + c * point.y;
            const double px = tx + rx, py = ty + ry;
            double nx = c * point.normalX - s * point.normalY;
            double ny = s * point.normalX + c * point.normalY;
            const double nn = std::hypot(nx, ny);
            if (nn < 1e-6) continue;
            nx /= nn; ny /= nn;

            int bestStep = 0;
            float bestMagnitude = 0.0f;
            for (int step = -static_cast<int>(std::ceil(radius));
                 step <= static_cast<int>(std::ceil(radius)); ++step) {
                const float x = static_cast<float>(px + step * nx);
                const float y = static_cast<float>(py + step * ny);
                const float magnitude = bilinear(edges.magnitude, x, y);
                if (magnitude <= bestMagnitude) continue;
                const float gx = bilinear(edges.gx, x, y), gy = bilinear(edges.gy, x, y);
                const float gn = std::hypot(gx, gy);
                if (gn < model.parameters.gradientLow) continue;
                if (std::abs((gx * nx + gy * ny) / gn) < cosTolerance) continue;
                bestMagnitude = magnitude; bestStep = step;
            }
            if (bestMagnitude <= 0.0f) continue;
            const float mm = bilinear(edges.magnitude, static_cast<float>(px + (bestStep - 1) * nx),
                                      static_cast<float>(py + (bestStep - 1) * ny));
            const float mp = bilinear(edges.magnitude, static_cast<float>(px + (bestStep + 1) * nx),
                                      static_cast<float>(py + (bestStep + 1) * ny));
            const float denominator = mm - 2.0f * bestMagnitude + mp;
            const double subpixel = std::abs(denominator) > 1e-5f
                ? std::clamp(0.5 * static_cast<double>(mm - mp) / denominator, -0.5, 0.5) : 0.0;
            const double ex = px + (bestStep + subpixel) * nx;
            const double ey = py + (bestStep + subpixel) * ny;
            double mx = bilinear(edges.gx, static_cast<float>(ex), static_cast<float>(ey));
            double my = bilinear(edges.gy, static_cast<float>(ex), static_cast<float>(ey));
            const double mn = std::hypot(mx, my);
            if (mn < model.parameters.gradientLow) continue;
            mx /= mn; my /= mn;
            if (mx * nx + my * ny < 0.0) { mx = -mx; my = -my; }

            const double residual = mx * (px - ex) + my * (py - ey);
            const double absResidual = std::abs(residual);
            const double huber = absResidual <= p.subpixelHuberDelta
                ? 1.0 : p.subpixelHuberDelta / std::max(absResidual, 1e-9);
            const double quality = std::min(1.0, static_cast<double>(bestMagnitude) / 255.0);
            const double weight = std::max(1, static_cast<int>(point.weight)) * quality * huber;
            const double dThetaX = -s * point.x - c * point.y;
            const double dThetaY =  c * point.x - s * point.y;
            const cv::Vec3d j(mx, my, mx * dThetaX + my * dThetaY);
            for (int row = 0; row < 3; ++row) {
                g[row] += weight * j[row] * residual;
                for (int col = 0; col < 3; ++col) h(row, col) += weight * j[row] * j[col];
            }
            weightedResidual2 += weight * residual * residual;
            weightSum += weight;
            ++valid;
        }
        out.refinementIterations = iteration + 1;
        out.validCorrespondences = valid;
        out.visibleRatio = static_cast<float>(valid) / static_cast<float>(model.points.size());
        out.rmsResidual = weightSum > 0.0 ? static_cast<float>(std::sqrt(weightedResidual2 / weightSum)) : 0.0f;
        if (valid < minRequired) break;
        const double damping = 1e-4 * std::max(1.0, (h(0, 0) + h(1, 1) + h(2, 2)) / 3.0);
        h(0, 0) += damping; h(1, 1) += damping; h(2, 2) += damping;
        cv::Vec3d delta;
        if (!cv::solve(h, -g, delta, cv::DECOMP_CHOLESKY)) break;
        const double translationNorm = std::hypot(delta[0], delta[1]);
        if (translationNorm > p.subpixelMaxTranslationStep) {
            const double factor = p.subpixelMaxTranslationStep / translationNorm;
            delta[0] *= factor; delta[1] *= factor;
        }
        delta[2] = std::clamp(delta[2], -static_cast<double>(p.subpixelMaxAngleStepRadians),
                              static_cast<double>(p.subpixelMaxAngleStepRadians));
        tx += delta[0]; ty += delta[1]; theta = wrapToPi(theta + delta[2]);
        out.refined = true;
        if (std::hypot(delta[0], delta[1]) < 0.01 && std::abs(delta[2]) < 0.0002) break;
    }
    if (out.refined) {
        out.pose.x = tx; out.pose.y = ty; out.pose.theta = theta;
        const float refineQuality = out.visibleRatio * std::exp(-out.rmsResidual / 2.0f);
        // Refinement is evidence, not only a bonus. A poor fit must be allowed
        // to lower a strong coarse score so that final ranking reflects pose quality.
        out.score = 0.5f * initial.score + 0.5f * refineQuality;
    }
    return out;
}

} // namespace ShapeMatch
