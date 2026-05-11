#pragma once

#include "VisionTools/VisionTools_global.h"

#include <QImage>

namespace VisionTools {

class VISIONTOOLS_API ImageView
{
public:
    ImageView();
    explicit ImageView(const QImage& image);

    bool isValid() const;
    int width() const;
    int height() const;

    double sampleBilinear(double x, double y, bool* ok = nullptr) const;
    double sampleNearest(int x, int y, bool* ok = nullptr) const;

private:
    QImage m_grayImage;
};

} // namespace VisionTools
