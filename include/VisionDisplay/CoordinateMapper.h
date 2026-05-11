#pragma once

#include "VisionDisplay/VisionDisplay_global.h"

#include <QPointF>
#include <QSize>
#include <QSizeF>
#include <QTransform>

namespace VisionDisplay {

class VISIONDISPLAY_API CoordinateMapper
{
public:
    void setImageSize(int width, int height);
    void setViewSize(double width, double height);
    void setZoom(double zoom);
    void setOffset(double offsetX, double offsetY);

    QSize imageSize() const;
    QSizeF viewSize() const;
    double zoom() const;
    QPointF offset() const;

    QPointF imageToView(const QPointF& imagePoint) const;
    QPointF viewToImage(const QPointF& viewPoint) const;

    QTransform imageToViewTransform() const;
    QTransform viewToImageTransform() const;

    void fitToWindow();

private:
    QSize m_imageSize;
    QSizeF m_viewSize;
    double m_zoom = 1.0;
    QPointF m_offset = QPointF(0.0, 0.0);
};

} // namespace VisionDisplay
