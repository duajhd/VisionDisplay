#include "VisionTools/ImageView.h"

#include <QtGlobal>

#include <algorithm>
#include <cmath>

namespace VisionTools {

ImageView::ImageView() = default;

ImageView::ImageView(const QImage& image)
{
    if (image.isNull()) {
        return;
    }

    if (image.format() == QImage::Format_Grayscale8) {
        m_grayImage = image.copy();
    } else {
        m_grayImage = image.convertToFormat(QImage::Format_Grayscale8);
    }
}

bool ImageView::isValid() const
{
    return !m_grayImage.isNull();
}

int ImageView::width() const
{
    return m_grayImage.width();
}

int ImageView::height() const
{
    return m_grayImage.height();
}

double ImageView::sampleNearest(int x, int y, bool* ok) const
{
    if (!isValid() || x < 0 || y < 0 || x >= width() || y >= height()) {
        if (ok) {
            *ok = false;
        }
        return 0.0;
    }

    if (ok) {
        *ok = true;
    }
    return static_cast<double>(m_grayImage.constScanLine(y)[x]);
}

double ImageView::sampleBilinear(double x, double y, bool* ok) const
{
    if (!isValid() || x < 0.0 || y < 0.0 || x > static_cast<double>(width() - 1) || y > static_cast<double>(height() - 1)) {
        if (ok) {
            *ok = false;
        }
        return 0.0;
    }

    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int x1 = std::min(x0 + 1, width() - 1);
    const int y1 = std::min(y0 + 1, height() - 1);
    const double fx = x - x0;
    const double fy = y - y0;

    const uchar* row0 = m_grayImage.constScanLine(y0);
    const uchar* row1 = m_grayImage.constScanLine(y1);
    const double v00 = row0[x0];
    const double v10 = row0[x1];
    const double v01 = row1[x0];
    const double v11 = row1[x1];
    const double top = v00 * (1.0 - fx) + v10 * fx;
    const double bottom = v01 * (1.0 - fx) + v11 * fx;

    if (ok) {
        *ok = true;
    }
    return top * (1.0 - fy) + bottom * fy;
}

} // namespace VisionTools
