#pragma once

#include "shape_match/core/ShapeTemplateModel.h"

#include <opencv2/core.hpp>

#include <vector>

namespace ShapeMatch {

class EdgeImageData
{
public:
    cv::Size imageSize;
    cv::Mat edgeMap;
    cv::Mat gradX;
    cv::Mat gradY;
    cv::Mat gradMag;
    cv::Mat distanceMap;
    cv::Mat nearestEdgeX;
    cv::Mat nearestEdgeY;
    cv::Mat orientationMap;
    cv::Mat gradMagMap;
    bool hasDistanceField = false;
    std::vector<cv::Point2d> edgePoints;
    std::vector<cv::Point2d> edgeNormals;

    bool empty() const;
    bool isInside(const cv::Point2d& p) const;
    bool hasNearestEdgeField() const;

    bool findNearestEdgeFast(const cv::Point2d& predictedPt,
                             double searchRadiusPx,
                             cv::Point2d& matchedPt,
                             cv::Point2d& imageNormal,
                             double& gradientMagValue,
                             double& distance) const;

    bool findNearestEdgeLocalWindow(const cv::Point2d& predictedPt,
                                    double searchRadiusPx,
                                    cv::Point2d& matchedPt,
                                    cv::Point2d& imageNormal,
                                    double& gradientMagValue,
                                    double& distance) const;

    bool findNearestEdgeLinearScanDebugOnly(const cv::Point2d& predictedPt,
                                            double searchRadiusPx,
                                            cv::Point2d& matchedPt,
                                            cv::Point2d& imageNormal,
                                            double& gradientMagValue,
                                            double& distance) const;

    bool findNearestEdge(const cv::Point2d& predictedPt,
                         double searchRadiusPx,
                         cv::Point2d& matchedPt,
                         cv::Point2d& imageNormal,
                         double& gradientMagValue,
                         double& distance) const;

    static EdgeImageData createFromTemplateAndPose(const ShapeTemplateModel& model,
                                                   const MatchPose& pose,
                                                   cv::Size imageSize);
};

} // namespace ShapeMatch
