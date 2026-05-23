#pragma once

#include "shape_match/core/EdgeImageData.h"

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace ShapeMatch {

struct RoiDistanceField
{
    int id = -1;
    int level = 0;
    cv::Rect roi;
    cv::Rect roiLevel0;
    EdgeImageData localEdgeData;
    bool valid = false;
    double buildTimeMs = 0.0;
    int pixelCount = 0;
    int sourceCandidateCount = 0;
};

class RoiDistanceFieldSet
{
public:
    void clear();
    int add(RoiDistanceField field);
    const RoiDistanceField* findContaining(int level, const cv::Point2d& p) const;
    const RoiDistanceField* findBestForRoi(int level, const cv::Rect& roi) const;
    int size() const;
    int totalPixels() const;
    double totalBuildTimeMs() const;
    const std::vector<RoiDistanceField>& fields() const;

private:
    std::vector<RoiDistanceField> m_fields;
};

struct EdgeQueryStats
{
    int roiDistanceFieldQueryCount = 0;
    int fullDistanceFieldQueryCount = 0;
    int localWindowFallbackCount = 0;
    int missingFieldCount = 0;
};

struct EdgeQueryContext
{
    const EdgeImageData* fullEdgeData = nullptr;
    const RoiDistanceFieldSet* roiFields = nullptr;
    bool preferRoiField = true;
    bool allowFullField = true;
    bool allowLocalWindowFallback = true;
    mutable EdgeQueryStats stats;
};

bool queryNearestEdge(const EdgeQueryContext& ctx,
                      int level,
                      const cv::Point2d& globalPoint,
                      double searchRadiusPx,
                      cv::Point2d& matchedPtGlobal,
                      cv::Point2d& imageNormal,
                      double& gradientMag,
                      double& distance,
                      std::string* mode = nullptr);

} // namespace ShapeMatch
