#pragma once

#include "shape_match/core/ShapeMatchTypes.h"

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace ShapeMatch {

class ShapeTemplateModel
{
public:
    std::string templateId;
    std::vector<TemplatePoint> points;
    cv::Rect2d boundingBox;
    cv::Point2d origin;

    bool empty() const;
    int pointCount() const;
    void computeBoundingBox();

    static ShapeTemplateModel createSyntheticRectangle(const std::string& id,
                                                       double width,
                                                       double height,
                                                       int pointsPerEdge);
    static ShapeTemplateModel createSyntheticCircle(const std::string& id,
                                                    double radius,
                                                    int pointCount);
};

} // namespace ShapeMatch
