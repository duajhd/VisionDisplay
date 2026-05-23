#include "shape_match/overlay/VisionDisplayOverlayAdapter.h"

#include <QColor>
#include <QPointF>

namespace ShapeMatch {

namespace {

QColor qcolor(const std::string& color)
{
    return QColor(QString::fromStdString(color));
}

QPointF qpoint(const cv::Point2d& p)
{
    return QPointF(p.x, p.y);
}

} // namespace

VisionDisplay::VisionDisplayOverlayData VisionDisplayOverlayAdapter::toVisionDisplayOverlay(const ShapeMatchOverlayData& data)
{
    VisionDisplay::VisionDisplayOverlayData out;
    for (const ShapeMatchOverlayPolyline& polyline : data.polylines) {
        VisionDisplay::OverlayPolyline p;
        p.color = qcolor(polyline.color);
        p.width = polyline.width;
        p.label = QString::fromStdString(polyline.label);
        for (const cv::Point2d& point : polyline.points) {
            p.points.push_back(qpoint(point));
        }
        out.polylines.push_back(p);
    }
    for (const ShapeMatchOverlayLine& line : data.lines) {
        VisionDisplay::OverlayPolyline p;
        p.color = qcolor(line.color);
        p.width = line.width;
        p.points.push_back(qpoint(line.p0));
        p.points.push_back(qpoint(line.p1));
        out.polylines.push_back(p);
    }
    for (const ShapeMatchOverlayPoint& point : data.points) {
        VisionDisplay::OverlayPoint p;
        p.pos = qpoint(point.pos);
        p.color = qcolor(point.color);
        p.radius = point.radius;
        p.label = QString::fromStdString(point.label);
        out.points.push_back(p);
    }
    for (const ShapeMatchOverlayText& text : data.texts) {
        VisionDisplay::OverlayText t;
        t.pos = qpoint(text.pos);
        t.text = QString::fromStdString(text.text);
        t.color = qcolor(text.color);
        t.fontSize = text.fontSize;
        out.texts.push_back(t);
    }
    return out;
}

} // namespace ShapeMatch
