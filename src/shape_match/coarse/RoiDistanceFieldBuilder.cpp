#include "shape_match/coarse/RoiDistanceFieldBuilder.h"

#include "shape_match/coarse/DistanceFieldBuilder.h"

#include <algorithm>
#include <chrono>

namespace ShapeMatch {

namespace {

double elapsedMsSince(std::chrono::steady_clock::time_point t0)
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

cv::Rect clampRect(const cv::Rect& rect, const cv::Size& size)
{
    return rect & cv::Rect(0, 0, size.width, size.height);
}

cv::Rect expandToMinSize(cv::Rect rect, const cv::Size& imageSize, int minW, int minH)
{
    const int addW = std::max(0, minW - rect.width);
    const int addH = std::max(0, minH - rect.height);
    rect.x -= addW / 2;
    rect.y -= addH / 2;
    rect.width += addW;
    rect.height += addH;
    return clampRect(rect, imageSize);
}

double overlapRatio(const cv::Rect& a, const cv::Rect& b)
{
    const int inter = (a & b).area();
    const int denom = std::min(std::max(1, a.area()), std::max(1, b.area()));
    return static_cast<double>(inter) / static_cast<double>(denom);
}

cv::Mat cropOrEmpty(const cv::Mat& src, const cv::Rect& roi)
{
    if (src.empty()) {
        return {};
    }
    return src(roi).clone();
}

} // namespace

RoiDistanceFieldSet RoiDistanceFieldBuilder::buildForRegions(
    const EdgeImageData& fullLevelEdgeData,
    const std::vector<cv::Rect>& rois,
    int level,
    double levelScale,
    const CoarseMatchConfig::RoiDistanceFieldConfig& config) const
{
    RoiDistanceFieldSet out;
    if (fullLevelEdgeData.edgeMap.empty() || rois.empty()) {
        return out;
    }

    const std::vector<cv::Rect> normalized = normalizeRois(rois, fullLevelEdgeData.imageSize, level, config);
    CoarseMatchConfig dfConfig;
    dfConfig.enableDistanceFieldScoring = true;
    DistanceFieldBuilder builder;
    int totalPixels = 0;
    for (const cv::Rect& roi : normalized) {
        if (roi.empty()) {
            continue;
        }
        const int pixels = roi.area();
        if (totalPixels + pixels > config.maxTotalRoiPixelsForDistanceField) {
            break;
        }

        RoiDistanceField field;
        field.level = level;
        field.roi = roi;
        const double invScale = levelScale > 1e-12 ? 1.0 / levelScale : 1.0;
        field.roiLevel0 = cv::Rect(static_cast<int>(std::floor(roi.x * invScale)),
                                   static_cast<int>(std::floor(roi.y * invScale)),
                                   static_cast<int>(std::ceil(roi.width * invScale)),
                                   static_cast<int>(std::ceil(roi.height * invScale)));
        field.pixelCount = pixels;
        field.localEdgeData.imageSize = roi.size();
        field.localEdgeData.edgeMap = cropOrEmpty(fullLevelEdgeData.edgeMap, roi);
        field.localEdgeData.gradX = cropOrEmpty(fullLevelEdgeData.gradX, roi);
        field.localEdgeData.gradY = cropOrEmpty(fullLevelEdgeData.gradY, roi);
        field.localEdgeData.gradMag = cropOrEmpty(fullLevelEdgeData.gradMag, roi);

        const auto t0 = std::chrono::steady_clock::now();
        field.valid = builder.build(field.localEdgeData, dfConfig);
        field.buildTimeMs = elapsedMsSince(t0);
        if (field.valid) {
            totalPixels += pixels;
            out.add(std::move(field));
        }
    }
    return out;
}

std::vector<cv::Rect> RoiDistanceFieldBuilder::normalizeRois(
    const std::vector<cv::Rect>& rois,
    const cv::Size& imageSize,
    int level,
    const CoarseMatchConfig::RoiDistanceFieldConfig& config) const
{
    const int padding = level <= 0 ? config.roiPaddingPxLevel0 : config.roiPaddingPxLevel1;
    std::vector<cv::Rect> rects;
    rects.reserve(rois.size());
    for (cv::Rect roi : rois) {
        roi.x -= padding;
        roi.y -= padding;
        roi.width += padding * 2;
        roi.height += padding * 2;
        roi = expandToMinSize(clampRect(roi, imageSize), imageSize, config.minRoiWidth, config.minRoiHeight);
        roi.width = std::min(roi.width, config.maxRoiWidth);
        roi.height = std::min(roi.height, config.maxRoiHeight);
        roi = clampRect(roi, imageSize);
        if (!roi.empty()) {
            rects.push_back(roi);
        }
    }

    if (config.mergeOverlappingRois) {
        bool changed = true;
        while (changed) {
            changed = false;
            for (size_t i = 0; i < rects.size() && !changed; ++i) {
                for (size_t j = i + 1; j < rects.size(); ++j) {
                    const cv::Rect merged = rects[i] | rects[j];
                    if (overlapRatio(rects[i], rects[j]) >= config.roiMergeOverlapRatio
                        && merged.width <= config.maxRoiWidth
                        && merged.height <= config.maxRoiHeight) {
                        rects[i] = clampRect(merged, imageSize);
                        rects.erase(rects.begin() + static_cast<std::ptrdiff_t>(j));
                        changed = true;
                        break;
                    }
                }
            }
        }
    }

    if (static_cast<int>(rects.size()) > config.maxRoiDistanceFields) {
        rects.resize(static_cast<size_t>(std::max(0, config.maxRoiDistanceFields)));
    }
    return rects;
}

} // namespace ShapeMatch
