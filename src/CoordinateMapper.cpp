#include "VisionDisplay/CoordinateMapper.h"

#include <algorithm>

namespace VisionDisplay {

void CoordinateMapper::setImageSize(int width, int height)
{
    m_imageSize = QSize(width, height);
}

void CoordinateMapper::setViewSize(double width, double height)
{
    m_viewSize = QSizeF(width, height);
}

void CoordinateMapper::setZoom(double zoom)
{
    m_zoom = std::max(zoom, 0.000001);
}

void CoordinateMapper::setOffset(double offsetX, double offsetY)
{
    m_offset = QPointF(offsetX, offsetY);
}

QSize CoordinateMapper::imageSize() const
{
    return m_imageSize;
}

QSizeF CoordinateMapper::viewSize() const
{
    return m_viewSize;
}

double CoordinateMapper::zoom() const
{
    return m_zoom;
}

QPointF CoordinateMapper::offset() const
{
    return m_offset;
}

QPointF CoordinateMapper::imageToView(const QPointF& imagePoint) const
{
    return QPointF(imagePoint.x() * m_zoom + m_offset.x(),
                   imagePoint.y() * m_zoom + m_offset.y());
}

QPointF CoordinateMapper::viewToImage(const QPointF& viewPoint) const
{
    return QPointF((viewPoint.x() - m_offset.x()) / m_zoom,
                   (viewPoint.y() - m_offset.y()) / m_zoom);
}

QTransform CoordinateMapper::imageToViewTransform() const
{
    QTransform transform;
    transform.translate(m_offset.x(), m_offset.y());
    transform.scale(m_zoom, m_zoom);
    return transform;
}

QTransform CoordinateMapper::viewToImageTransform() const
{
    return imageToViewTransform().inverted();
}

void CoordinateMapper::fitToWindow()
{
    if (m_imageSize.isEmpty() || m_viewSize.isEmpty()) {
        m_zoom = 1.0;
        m_offset = QPointF(0.0, 0.0);
        return;
    }

    const double zoomX = m_viewSize.width() / static_cast<double>(m_imageSize.width());
    const double zoomY = m_viewSize.height() / static_cast<double>(m_imageSize.height());
    m_zoom = std::min(zoomX, zoomY);

    const QSizeF scaledSize(m_imageSize.width() * m_zoom, m_imageSize.height() * m_zoom);
    m_offset = QPointF((m_viewSize.width() - scaledSize.width()) * 0.5,
                       (m_viewSize.height() - scaledSize.height()) * 0.5);
}

} // namespace VisionDisplay
