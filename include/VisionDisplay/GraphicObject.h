#pragma once

#include "VisionDisplay/VisionDisplay_global.h"

#include <QColor>
#include <QLineF>
#include <QPointF>
#include <QRectF>
#include <QString>
#include <QVector>

namespace VisionDisplay {

enum class GraphicType {
    Line,
    Rect,
    Circle,
    Text,
    Cross,
    Polyline,
    DefectBox,
    DefectContour,
    BlobRegion,
    MatchContour,
    FittedLine,
    FittedCircle,
    StatusText,
    RotatedRect,
    Arc,
    Arrow,
    PointMarker
};

enum class DisplayLayer {
    Roi = 10,
    Result = 20,
    Measure = 30,
    Temporary = 40,
    Debug = 50
};

struct VISIONDISPLAY_API GraphicStyle
{
    QColor strokeColor = Qt::green;
    QColor fillColor = Qt::transparent;
    double lineWidth = 1.5;
    double opacity = 1.0;
    int fontPixelSize = 14;
    bool lineWidthInViewPixels = true;
    bool textSizeInViewPixels = true;
};

class VISIONDISPLAY_API GraphicObject
{
public:
    QString id;
    GraphicType type = GraphicType::Line;
    int layer = static_cast<int>(DisplayLayer::Result);
    bool visible = true;
    bool resultGraphic = false;
    bool toolGraphic = false;
    QString toolId;
    QString toolName;
    QString toolType;
    GraphicStyle style;

    QLineF line;
    QRectF rect;
    QPointF center;
    double radius = 0.0;
    double startAngleDeg = 0.0;
    double spanAngleDeg = 360.0;
    double angleDeg = 0.0;
    QPointF textPosition;
    QString text;
    QPointF crossCenter;
    double crossSize = 0.0;
    QVector<QPointF> points;
    QString label;
    bool statusOk = true;
    double markerSize = 7.0;
    double arrowHeadSize = 10.0;
};

} // namespace VisionDisplay
