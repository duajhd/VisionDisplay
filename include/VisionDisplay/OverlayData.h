#pragma once

#include "VisionDisplay/VisionDisplay_global.h"

#include <QColor>
#include <QPointF>
#include <QString>
#include <QVector>

namespace VisionDisplay {

struct VISIONDISPLAY_API OverlayPoint
{
    QPointF pos;
    QColor color = Qt::green;
    double radius = 4.0;
    QString label;
};

struct VISIONDISPLAY_API OverlayPolyline
{
    QVector<QPointF> points;
    QColor color = Qt::green;
    double width = 1.5;
    QString label;
};

struct VISIONDISPLAY_API OverlayArrow
{
    QPointF p0;
    QPointF p1;
    QColor color = QColor(80, 220, 255);
    double width = 1.5;
};

struct VISIONDISPLAY_API OverlayText
{
    QPointF pos;
    QString text;
    QColor color = Qt::white;
    int fontSize = 13;
};

struct VISIONDISPLAY_API VisionDisplayOverlayData
{
    QVector<OverlayPoint> points;
    QVector<OverlayPolyline> polylines;
    QVector<OverlayArrow> arrows;
    QVector<OverlayText> texts;

    void clear();
    bool empty() const;
};

} // namespace VisionDisplay
