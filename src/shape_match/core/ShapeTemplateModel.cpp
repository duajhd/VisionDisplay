#include "shape_match/core/ShapeTemplateModel.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ShapeMatch {

bool ShapeTemplateModel::empty() const
{
    return points.empty();
}

int ShapeTemplateModel::pointCount() const
{
    return static_cast<int>(points.size());
}

void ShapeTemplateModel::computeBoundingBox()
{
    if (points.empty()) {
        boundingBox = cv::Rect2d();
        return;
    }

    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double maxY = std::numeric_limits<double>::lowest();
    for (const TemplatePoint& p : points) {
        minX = std::min(minX, p.position.x);
        minY = std::min(minY, p.position.y);
        maxX = std::max(maxX, p.position.x);
        maxY = std::max(maxY, p.position.y);
    }
    boundingBox = cv::Rect2d(minX, minY, maxX - minX, maxY - minY);
}

ShapeTemplateModel ShapeTemplateModel::createSyntheticRectangle(const std::string& id,
                                                                double width,
                                                                double height,
                                                                int pointsPerEdge)
{
    ShapeTemplateModel model;
    model.templateId = id;
    model.origin = cv::Point2d(0.0, 0.0);
    pointsPerEdge = std::max(2, pointsPerEdge);
    model.points.reserve(static_cast<size_t>(pointsPerEdge * 4));

    const double hw = width * 0.5;
    const double hh = height * 0.5;
    int pointId = 0;
    auto addPoint = [&](const cv::Point2d& pos, const cv::Point2d& normal, const cv::Point2d& tangent, int chainId) {
        TemplatePoint p;
        p.id = pointId++;
        p.position = pos;
        p.normal = normalized(normal);
        p.tangent = normalized(tangent);
        p.gradientDir = p.normal;
        p.gradientMag = 255.0;
        p.weight = 1.0;
        p.chainId = chainId;
        p.polarity = EdgePolarity::DarkToBright;
        model.points.push_back(p);
    };

    for (int i = 0; i < pointsPerEdge; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(pointsPerEdge - 1);
        addPoint(cv::Point2d(-hw + width * t, -hh), cv::Point2d(0.0, -1.0), cv::Point2d(1.0, 0.0), 0);
        addPoint(cv::Point2d(hw, -hh + height * t), cv::Point2d(1.0, 0.0), cv::Point2d(0.0, 1.0), 1);
        addPoint(cv::Point2d(hw - width * t, hh), cv::Point2d(0.0, 1.0), cv::Point2d(-1.0, 0.0), 2);
        addPoint(cv::Point2d(-hw, hh - height * t), cv::Point2d(-1.0, 0.0), cv::Point2d(0.0, -1.0), 3);
    }

    model.computeBoundingBox();
    return model;
}

ShapeTemplateModel ShapeTemplateModel::createSyntheticCircle(const std::string& id,
                                                             double radius,
                                                             int pointCountValue)
{
    ShapeTemplateModel model;
    model.templateId = id;
    model.origin = cv::Point2d(0.0, 0.0);
    pointCountValue = std::max(8, pointCountValue);
    model.points.reserve(static_cast<size_t>(pointCountValue));
    for (int i = 0; i < pointCountValue; ++i) {
        const double a = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(pointCountValue);
        const cv::Point2d n(std::cos(a), std::sin(a));
        TemplatePoint p;
        p.id = i;
        p.position = cv::Point2d(radius * n.x, radius * n.y);
        p.normal = n;
        p.tangent = cv::Point2d(-n.y, n.x);
        p.gradientDir = n;
        p.gradientMag = 255.0;
        p.weight = 1.0;
        p.chainId = 0;
        p.polarity = EdgePolarity::DarkToBright;
        model.points.push_back(p);
    }
    model.computeBoundingBox();
    return model;
}

} // namespace ShapeMatch
