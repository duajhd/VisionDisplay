#pragma once

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace ShapeMatch {

struct ShapeMatchOverlayPoint
{
    cv::Point2d pos;
    std::string color = "#00ff00";
    double radius = 4.0;
    std::string label;
};

struct ShapeMatchOverlayPolyline
{
    std::vector<cv::Point2d> points;
    std::string color = "#00ff00";
    double width = 1.5;
    std::string label;
};

struct ShapeMatchOverlayLine
{
    cv::Point2d p0;
    cv::Point2d p1;
    std::string color = "#ffffff";
    double width = 1.0;
};

struct ShapeMatchOverlayText
{
    cv::Point2d pos;
    std::string text;
    std::string color = "#ffffff";
    int fontSize = 13;
};

struct ShapeMatchOverlayData
{
    std::vector<ShapeMatchOverlayPoint> points;
    std::vector<ShapeMatchOverlayPolyline> polylines;
    std::vector<ShapeMatchOverlayLine> lines;
    std::vector<ShapeMatchOverlayText> texts;

    bool empty() const
    {
        return points.empty() && polylines.empty() && lines.empty() && texts.empty();
    }
};

} // namespace ShapeMatch
