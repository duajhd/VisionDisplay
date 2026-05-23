#include "VisionTools/Matching/ImagePyramid.h"

#include <algorithm>
#include <utility>

namespace VisionTools::Matching {

GrayImageLevel ImagePyramid::fromQImage(const QImage& image)
{
    GrayImageLevel level;
    if (image.isNull()) {
        return level;
    }

    const QImage grayImage = image.format() == QImage::Format_Grayscale8
        ? image
        : image.convertToFormat(QImage::Format_Grayscale8);

    level.width = grayImage.width();
    level.height = grayImage.height();
    level.scale = 1.0;
    level.pixels.resize(static_cast<size_t>(level.width * level.height));

    for (int y = 0; y < level.height; ++y) {
        const uchar* row = grayImage.constScanLine(y);
        for (int x = 0; x < level.width; ++x) {
            level.pixels[static_cast<size_t>(y * level.width + x)] = static_cast<double>(row[x]);
        }
    }

    return level;
}

std::vector<GrayImageLevel> ImagePyramid::build(const QImage& image, int levels)
{
    std::vector<GrayImageLevel> pyramid;
    levels = std::max(1, levels);
    pyramid.reserve(static_cast<size_t>(levels));

    GrayImageLevel current = fromQImage(image);
    if (!current.isValid()) {
        return pyramid;
    }
    pyramid.push_back(current);

    for (int levelIndex = 1; levelIndex < levels; ++levelIndex) {
        const GrayImageLevel& previous = pyramid.back();
        const int nextWidth = previous.width / 2;
        const int nextHeight = previous.height / 2;
        if (nextWidth < 1 || nextHeight < 1) {
            break;
        }

        GrayImageLevel next;
        next.width = nextWidth;
        next.height = nextHeight;
        next.scale = previous.scale * 0.5;
        next.pixels.resize(static_cast<size_t>(next.width * next.height));

        for (int y = 0; y < next.height; ++y) {
            for (int x = 0; x < next.width; ++x) {
                const int sx = x * 2;
                const int sy = y * 2;
                const double p00 = previous.pixels[static_cast<size_t>(sy * previous.width + sx)];
                const double p01 = previous.pixels[static_cast<size_t>(sy * previous.width + std::min(sx + 1, previous.width - 1))];
                const double p10 = previous.pixels[static_cast<size_t>(std::min(sy + 1, previous.height - 1) * previous.width + sx)];
                const double p11 = previous.pixels[static_cast<size_t>(std::min(sy + 1, previous.height - 1) * previous.width + std::min(sx + 1, previous.width - 1))];
                next.pixels[static_cast<size_t>(y * next.width + x)] = (p00 + p01 + p10 + p11) * 0.25;
            }
        }

        pyramid.push_back(std::move(next));
    }

    return pyramid;
}

} // namespace VisionTools::Matching
