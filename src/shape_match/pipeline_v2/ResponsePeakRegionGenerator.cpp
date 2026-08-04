#include "shape_match/pipeline_v2/ResponsePeakRegionGenerator.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace ShapeMatch {

namespace {

double elapsed(std::chrono::steady_clock::time_point start)
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

double scaleOfLevel(const ShapeMatchPipelineV2Context& context, int level)
{
    if (context.imagePyramid != nullptr && level >= 0 && level < context.imagePyramid->levelCount()) {
        return context.imagePyramid->scaleOfLevel(level);
    }
    return 1.0;
}

cv::Size level0Size(const ShapeMatchPipelineV2Context& context)
{
    if (context.edgeData != nullptr) {
        return context.edgeData->imageSize;
    }
    if (context.imagePyramid != nullptr && context.imagePyramid->levelCount() > 0) {
        return context.imagePyramid->level(0).imageSize;
    }
    return cv::Size();
}

cv::Rect clampRect(cv::Rect rect, cv::Size size)
{
    rect &= cv::Rect(0, 0, size.width, size.height);
    return rect;
}

bool overlapsEnough(const cv::Rect& a, const cv::Rect& b)
{
    const int interArea = (a & b).area();
    if (interArea <= 0) {
        return false;
    }
    const int minArea = std::max(1, std::min(a.area(), b.area()));
    return static_cast<double>(interArea) / static_cast<double>(minArea) > 0.35;
}

} // namespace

std::vector<ResponseCandidateRegion> ResponsePeakRegionGenerator::generateRegions(const std::vector<VerifiedResponsePeak>& peaks,
                                                                                 const ShapeMatchPipelineV2Context& context,
                                                                                 const ShapeMatchPipelineV2Config& config,
                                                                                 double* elapsedMs) const
{
    const auto start = std::chrono::steady_clock::now();
    std::vector<ResponseCandidateRegion> regions;
    if (!config.responsePipeline.enableResponsePeakRegions || context.templateModel == nullptr) {
        if (elapsedMs != nullptr) {
            *elapsedMs = elapsed(start);
        }
        return regions;
    }

    const int responseLevel = config.responsePipeline.responsePyramidLevel;
    const double scale = std::max(1e-9, scaleOfLevel(context, responseLevel));
    const cv::Size image0 = level0Size(context);
    const cv::Rect2d bbox = context.templateModel->boundingBox;
    const int pad = std::max(0, config.responsePipeline.regionPaddingPxLevel0);
    int totalPixels = 0;

    for (const VerifiedResponsePeak& peak : peaks) {
        if (static_cast<int>(regions.size()) >= config.responsePipeline.maxResponseRegions) {
            break;
        }
        MatchPose pose0 = peak.pose;
        pose0.x /= scale;
        pose0.y /= scale;

        const int w = std::max(16, static_cast<int>(std::ceil(bbox.width + pad * 2)));
        const int h = std::max(16, static_cast<int>(std::ceil(bbox.height + pad * 2)));
        cv::Rect roi0(static_cast<int>(std::round(pose0.x - w * 0.5)),
                      static_cast<int>(std::round(pose0.y - h * 0.5)),
                      w,
                      h);
        roi0 = clampRect(roi0, image0);
        if (roi0.empty()) {
            continue;
        }

        bool merged = false;
        for (ResponseCandidateRegion& region : regions) {
            if (overlapsEnough(region.roiLevel0, roi0)) {
                region.roiLevel0 |= roi0;
                region.sourcePeakCount += 1;
                region.seedScore = std::max(region.seedScore, peak.combinedScore);
                merged = true;
                break;
            }
        }
        if (merged) {
            continue;
        }
        if (totalPixels + roi0.area() > config.responsePipeline.maxTotalRegionPixels) {
            continue;
        }

        ResponseCandidateRegion region;
        region.regionId = static_cast<int>(regions.size());
        region.roiLevel0 = roi0;
        region.roiAtResponseLevel = cv::Rect(static_cast<int>(std::round(roi0.x * scale)),
                                             static_cast<int>(std::round(roi0.y * scale)),
                                             static_cast<int>(std::round(roi0.width * scale)),
                                             static_cast<int>(std::round(roi0.height * scale)));
        region.seedPoseAtResponseLevel = peak.pose;
        region.seedPoseLevel0 = pose0;
        region.seedScore = peak.combinedScore;
        region.sourcePeakCount = 1;
        regions.push_back(region);
        totalPixels += roi0.area();
    }

    for (size_t i = 0; i < regions.size(); ++i) {
        regions[i].regionId = static_cast<int>(i);
    }

    if (elapsedMs != nullptr) {
        *elapsedMs = elapsed(start);
    }
    return regions;
}

} // namespace ShapeMatch
