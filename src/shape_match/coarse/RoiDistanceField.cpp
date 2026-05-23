#include "shape_match/coarse/RoiDistanceField.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ShapeMatch {

void RoiDistanceFieldSet::clear()
{
    m_fields.clear();
}

int RoiDistanceFieldSet::add(RoiDistanceField field)
{
    field.id = static_cast<int>(m_fields.size());
    m_fields.push_back(std::move(field));
    return m_fields.back().id;
}

const RoiDistanceField* RoiDistanceFieldSet::findContaining(int level, const cv::Point2d& p) const
{
    const RoiDistanceField* best = nullptr;
    int bestArea = std::numeric_limits<int>::max();
    const cv::Point pi(static_cast<int>(std::round(p.x)), static_cast<int>(std::round(p.y)));
    for (const RoiDistanceField& field : m_fields) {
        if (!field.valid || field.level != level || !field.roi.contains(pi)) {
            continue;
        }
        const int area = field.roi.area();
        if (!best || area < bestArea) {
            best = &field;
            bestArea = area;
        }
    }
    return best;
}

const RoiDistanceField* RoiDistanceFieldSet::findBestForRoi(int level, const cv::Rect& roi) const
{
    const RoiDistanceField* best = nullptr;
    int bestArea = std::numeric_limits<int>::max();
    for (const RoiDistanceField& field : m_fields) {
        if (!field.valid || field.level != level) {
            continue;
        }
        if ((field.roi & roi) != roi) {
            continue;
        }
        const int area = field.roi.area();
        if (!best || area < bestArea) {
            best = &field;
            bestArea = area;
        }
    }
    return best;
}

int RoiDistanceFieldSet::size() const
{
    return static_cast<int>(m_fields.size());
}

int RoiDistanceFieldSet::totalPixels() const
{
    int total = 0;
    for (const RoiDistanceField& field : m_fields) {
        total += field.pixelCount;
    }
    return total;
}

double RoiDistanceFieldSet::totalBuildTimeMs() const
{
    double total = 0.0;
    for (const RoiDistanceField& field : m_fields) {
        total += field.buildTimeMs;
    }
    return total;
}

const std::vector<RoiDistanceField>& RoiDistanceFieldSet::fields() const
{
    return m_fields;
}

bool queryNearestEdge(const EdgeQueryContext& ctx,
                      int level,
                      const cv::Point2d& globalPoint,
                      double searchRadiusPx,
                      cv::Point2d& matchedPtGlobal,
                      cv::Point2d& imageNormal,
                      double& gradientMag,
                      double& distance,
                      std::string* mode)
{
    if (ctx.preferRoiField && ctx.roiFields) {
        const RoiDistanceField* field = ctx.roiFields->findContaining(level, globalPoint);
        if (field && field->valid) {
            const cv::Point2d localPoint(globalPoint.x - field->roi.x, globalPoint.y - field->roi.y);
            cv::Point2d matchedLocal;
            if (field->localEdgeData.findNearestEdgeFast(localPoint,
                                                         searchRadiusPx,
                                                         matchedLocal,
                                                         imageNormal,
                                                         gradientMag,
                                                         distance)) {
                matchedPtGlobal = cv::Point2d(matchedLocal.x + field->roi.x, matchedLocal.y + field->roi.y);
                ++ctx.stats.roiDistanceFieldQueryCount;
                if (mode) {
                    *mode = "roi_distance_field";
                }
                return true;
            }
        }
    }

    if (ctx.allowFullField && ctx.fullEdgeData && ctx.fullEdgeData->hasNearestEdgeField()) {
        if (ctx.fullEdgeData->findNearestEdgeFast(globalPoint,
                                                  searchRadiusPx,
                                                  matchedPtGlobal,
                                                  imageNormal,
                                                  gradientMag,
                                                  distance)) {
            ++ctx.stats.fullDistanceFieldQueryCount;
            if (mode) {
                *mode = "full_distance_field";
            }
            return true;
        }
    }

    if (ctx.allowLocalWindowFallback && ctx.fullEdgeData) {
        if (ctx.fullEdgeData->findNearestEdgeLocalWindow(globalPoint,
                                                         searchRadiusPx,
                                                         matchedPtGlobal,
                                                         imageNormal,
                                                         gradientMag,
                                                         distance)) {
            ++ctx.stats.localWindowFallbackCount;
            if (mode) {
                *mode = "local_window";
            }
            return true;
        }
    }

    ++ctx.stats.missingFieldCount;
    if (mode) {
        *mode = "missing";
    }
    return false;
}

} // namespace ShapeMatch
