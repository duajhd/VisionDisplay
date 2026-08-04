#include "shape_match/pipeline_v3/ResponseMapBuilderV3.h"

#include "shape_match/core/ShapeTemplateModel.h"
#include "shape_match/pipeline_v3/AngleViewBuilderV3.h"
#include "shape_match/pipeline_v3/WorkerPoolV3.h"
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ShapeMatch {
namespace {

cv::Mat toGray8(const cv::Mat& src)
{
    if (src.empty()) return {};
    cv::Mat gray;
    if (src.channels() == 1) gray = src;
    else if (src.channels() == 3) cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    else if (src.channels() == 4) cv::cvtColor(src, gray, cv::COLOR_BGRA2GRAY);
    else throw std::invalid_argument("ShapeMatch V3 supports 1, 3, or 4 channel images");
    if (gray.depth() != CV_8U) {
        cv::Mat converted;
        gray.convertTo(converted, CV_8U);
        return converted;
    }
    return gray;
}

} // namespace

ResponseMapV3 ResponseMapBuilderV3::build(const cv::Mat& image, float low, float high,
                                          bool nonMaximumSuppression,
                                          WorkerPoolV3* workerPool) const
{
    if (!(high > low) || low < 0.0f) throw std::invalid_argument("invalid response thresholds");
    cv::Mat gray = toGray8(image);
    if (gray.empty()) return {};
    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(3, 3), 0.8, 0.8, cv::BORDER_REPLICATE);
    cv::Mat edgeMask;
    cv::Canny(blurred, edgeMask, low, high, 3, true);

    cv::Mat maxima;
    if (nonMaximumSuppression) {
        cv::Mat gx, gy, magnitude;
        cv::Scharr(blurred, gx, CV_32F, 1, 0);
        cv::Scharr(blurred, gy, CV_32F, 0, 1);
        cv::magnitude(gx, gy, magnitude);
        cv::Mat dilated;
        cv::dilate(magnitude, dilated, cv::Mat());
        maxima = magnitude == dilated;
    }
    ResponseMapV3 out;
    out.width = gray.cols;
    out.height = gray.rows;
    out.orientationBinCount = 1;
    out.maxDistancePx = 0.0f;
    out.binMaps[0].create(edgeMask.size(), CV_8UC1);
    auto buildRows = [&](int rowBegin, int rowEnd, int) {
        for (int y = rowBegin; y < rowEnd; ++y) {
            const std::uint8_t* edges = edgeMask.ptr<std::uint8_t>(y);
            const std::uint8_t* localMaxima = maxima.empty() ? nullptr : maxima.ptr<std::uint8_t>(y);
            std::uint8_t* response = out.binMaps[0].ptr<std::uint8_t>(y);
            for (int x = 0; x < edgeMask.cols; ++x) {
                const std::uint8_t edge = localMaxima ? static_cast<std::uint8_t>(edges[x] & localMaxima[x])
                                                      : edges[x];
                response[x] = static_cast<std::uint8_t>(~edge);
            }
        }
    };
    if (workerPool) workerPool->parallelFor(0, edgeMask.rows, workerPool->threadCount(), buildRows);
    else buildRows(0, edgeMask.rows, 0);
    out.stride = static_cast<int>(out.binMaps[0].step[0]);
    out.planeStride = out.stride * out.height;
    out.binData[0] = out.binMaps[0].ptr<std::uint8_t>();
    out.distanceScale = 1.0f;
    return out;
}

ShapeModelV3 ShapeModelTrainerV3::createModel(const cv::Mat& templateImage,
                                               const cv::Mat& templateMask,
                                               const ShapeModelParametersV3& parameters) const
{
    if (parameters.templatePointMinDistance < 0.0f || parameters.templateGradientThreshold < 0.0f)
        throw std::invalid_argument("invalid V3 template thresholds");
    cv::Mat gray = toGray8(templateImage);
    if (gray.empty()) throw std::invalid_argument("empty template image");
    if (!templateMask.empty() && (templateMask.size() != gray.size() || templateMask.type() != CV_8UC1))
        throw std::invalid_argument("template mask must be CV_8UC1 and match template size");

    cv::Mat gx, gy, mag;
    cv::Scharr(gray, gx, CV_32F, 1, 0, 1.0 / 16.0);
    cv::Scharr(gray, gy, CV_32F, 0, 1, 1.0 / 16.0);
    cv::magnitude(gx, gy, mag);
    ShapeModelV3 model;
    model.templateSize = gray.size();
    model.origin = cv::Point2f((gray.cols - 1) * 0.5f, (gray.rows - 1) * 0.5f);
    model.parameters = parameters;
    const float minD2 = std::max(0.0f, parameters.templatePointMinDistance);
    const int cell = std::max(1, static_cast<int>(std::ceil(minD2)));
    for (int y = 0; y < gray.rows; y += cell) {
        for (int x = 0; x < gray.cols; x += cell) {
            if (!templateMask.empty() && templateMask.at<std::uint8_t>(y, x) == 0) continue;
            float best = parameters.templateGradientThreshold;
            cv::Point bestPt(-1, -1);
            for (int yy = y; yy < std::min(y + cell, gray.rows); ++yy)
                for (int xx = x; xx < std::min(x + cell, gray.cols); ++xx)
                    if (mag.at<float>(yy, xx) > best) { best = mag.at<float>(yy, xx); bestPt = {xx, yy}; }
            if (bestPt.x >= 0) {
                ShapePointV3 p;
                p.x = bestPt.x - model.origin.x;
                p.y = bestPt.y - model.origin.y;
                p.quality = best;
                p.gradientAngleRadians = std::atan2(gy.at<float>(bestPt.y, bestPt.x),
                                                    gx.at<float>(bestPt.y, bestPt.x));
                p.normalX = std::cos(p.gradientAngleRadians);
                p.normalY = std::sin(p.gradientAngleRadians);
                p.orientationBin = 0;
                p.weight = static_cast<std::uint8_t>(std::clamp(1 + static_cast<int>(best / 64.0f), 1, 4));
                model.points.push_back(p);
            }
        }
    }
    // Interleave quality-sorted spatial cells. Early stages therefore cover the
    // full template instead of collapsing onto one locally strong contour.
    std::array<std::vector<ShapePointV3>, 16> cells;
    for (const ShapePointV3& p : model.points) {
        const int cx = std::clamp(static_cast<int>((p.x + model.origin.x) * 4.0f / gray.cols), 0, 3);
        const int cy = std::clamp(static_cast<int>((p.y + model.origin.y) * 4.0f / gray.rows), 0, 3);
        cells[static_cast<size_t>(cy * 4 + cx)].push_back(p);
    }
    for (auto& points : cells)
        std::stable_sort(points.begin(), points.end(),
                         [](const ShapePointV3& a, const ShapePointV3& b) { return a.quality > b.quality; });
    model.points.clear();
    for (size_t rank = 0;; ++rank) {
        bool added = false;
        for (auto& points : cells) {
            if (rank < points.size()) { model.points.push_back(points[rank]); added = true; }
        }
        if (!added) break;
    }
    AngleViewBuilderV3().precompute(model);
    return model;
}

ShapeModelV3 ShapeModelTrainerV3::fromTemplateModel(const ShapeTemplateModel& source,
                                                     const ShapeModelParametersV3& parameters) const
{
    if (source.empty()) throw std::invalid_argument("empty ShapeTemplateModel");
    ShapeModelV3 model;
    model.parameters = parameters;
    model.origin = cv::Point2f(static_cast<float>(source.origin.x), static_cast<float>(source.origin.y));
    model.templateSize = cv::Size(static_cast<int>(std::ceil(source.boundingBox.width)),
                                  static_cast<int>(std::ceil(source.boundingBox.height)));
    model.points.reserve(source.points.size());
    for (const TemplatePoint& src : source.points) {
        ShapePointV3 p;
        // ShapeTemplateModel points are already expressed relative to the pose
        // origin; source.origin is the pose's image-space training location.
        p.x = static_cast<float>(src.position.x);
        p.y = static_cast<float>(src.position.y);
        p.quality = static_cast<float>(std::max(src.gradientMag, src.weight));
        const cv::Point2d direction = norm(src.gradientDir) > 1e-6 ? src.gradientDir : src.normal;
        p.gradientAngleRadians = static_cast<float>(std::atan2(direction.y, direction.x));
        p.normalX = static_cast<float>(direction.x);
        p.normalY = static_cast<float>(direction.y);
        p.orientationBin = 0;
        p.weight = static_cast<std::uint8_t>(std::clamp(static_cast<int>(std::lround(src.weight)), 1, 255));
        model.points.push_back(p);
    }
    std::stable_sort(model.points.begin(), model.points.end(),
                     [](const ShapePointV3& a, const ShapePointV3& b) { return a.quality > b.quality; });
    AngleViewBuilderV3().precompute(model);
    return model;
}

} // namespace ShapeMatch
