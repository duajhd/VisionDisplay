#include "VisionTools/Matching/GrayMatcher.h"

#include "VisionTools/Matching/ImagePyramid.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace VisionTools::Matching {

namespace {

QRect normalizedRoi(QRect roi, const QRect& bounds)
{
    if (roi.isNull() || roi.isEmpty()) {
        roi = bounds;
    }
    return roi.normalized().intersected(bounds);
}

double parabolicOffset(double minus, double center, double plus)
{
    const double denominator = minus - 2.0 * center + plus;
    if (std::abs(denominator) <= 1.0e-12) {
        return 0.0;
    }
    return std::clamp(0.5 * (minus - plus) / denominator, -1.0, 1.0);
}

} // namespace

GrayTemplate GrayMatcher::buildTemplate(const QImage& image, QRect roi) const
{
    GrayTemplate tmpl;
    const GrayImageLevel gray = ImagePyramid::fromQImage(image);
    if (!gray.isValid()) {
        tmpl.message = QStringLiteral("Invalid image");
        return tmpl;
    }

    roi = normalizedRoi(roi, QRect(0, 0, gray.width, gray.height));
    if (roi.width() < 2 || roi.height() < 2) {
        tmpl.message = QStringLiteral("Invalid template ROI");
        return tmpl;
    }

    tmpl.width = roi.width();
    tmpl.height = roi.height();
    tmpl.originX = (static_cast<double>(tmpl.width) - 1.0) * 0.5;
    tmpl.originY = (static_cast<double>(tmpl.height) - 1.0) * 0.5;
    tmpl.gray.resize(static_cast<size_t>(tmpl.width * tmpl.height));

    double sum = 0.0;
    for (int y = 0; y < tmpl.height; ++y) {
        for (int x = 0; x < tmpl.width; ++x) {
            const double value = gray.pixels[static_cast<size_t>((roi.y() + y) * gray.width + roi.x() + x)];
            tmpl.gray[static_cast<size_t>(y * tmpl.width + x)] = value;
            sum += value;
        }
    }

    const double count = static_cast<double>(tmpl.gray.size());
    tmpl.mean = count > 0.0 ? sum / count : 0.0;

    double normSq = 0.0;
    for (double value : tmpl.gray) {
        const double centered = value - tmpl.mean;
        normSq += centered * centered;
    }
    tmpl.norm = std::sqrt(normSq);
    tmpl.message = tmpl.norm > 0.0 ? QStringLiteral("OK") : QStringLiteral("Template has no grayscale variation");
    return tmpl;
}

GrayMatchResult GrayMatcher::match(const QImage& searchImage,
                                   const GrayTemplate& tmpl,
                                   const GrayMatchParams& params) const
{
    GrayMatchResult result;
    if (!tmpl.isValid()) {
        result.message = QStringLiteral("Invalid gray template");
        return result;
    }

    const GrayImageLevel gray = ImagePyramid::fromQImage(searchImage);
    if (!gray.isValid()) {
        result.message = QStringLiteral("Invalid search image");
        return result;
    }

    QRect searchRoi = normalizedRoi(params.searchRoi, QRect(0, 0, gray.width, gray.height));
    if (searchRoi.width() < tmpl.width || searchRoi.height() < tmpl.height) {
        result.message = QStringLiteral("Search ROI is smaller than template");
        return result;
    }

    const int stepX = std::max(1, params.stepX);
    const int stepY = std::max(1, params.stepY);
    const int topK = std::max(1, params.topK);
    const int maxX = searchRoi.right() - tmpl.width + 1;
    const int maxY = searchRoi.bottom() - tmpl.height + 1;

    std::vector<GrayMatchCandidate> candidates;
    candidates.reserve(static_cast<size_t>(topK));

    for (int y = searchRoi.y(); y <= maxY; y += stepY) {
        for (int x = searchRoi.x(); x <= maxX; x += stepX) {
            const double score = scoreAt(gray.pixels, gray.width, gray.height, tmpl, x, y);
            if (!std::isfinite(score)) {
                continue;
            }

            candidates.push_back({static_cast<double>(x), static_cast<double>(y), score});
            std::sort(candidates.begin(), candidates.end(), [](const GrayMatchCandidate& a, const GrayMatchCandidate& b) {
                return a.score > b.score;
            });
            if (candidates.size() > static_cast<size_t>(topK)) {
                candidates.pop_back();
            }
        }
    }

    if (candidates.empty()) {
        result.message = QStringLiteral("No valid match candidate");
        return result;
    }

    result.candidates = candidates;
    result.x = candidates.front().x;
    result.y = candidates.front().y;
    result.score = candidates.front().score;

    if (params.enableSubpixelRefine) {
        const int ix = static_cast<int>(std::round(result.x));
        const int iy = static_cast<int>(std::round(result.y));
        if (ix > searchRoi.left()
            && iy > searchRoi.top()
            && ix < maxX
            && iy < maxY) {
            const double center = scoreAt(gray.pixels, gray.width, gray.height, tmpl, ix, iy);
            const double left = scoreAt(gray.pixels, gray.width, gray.height, tmpl, ix - 1, iy);
            const double right = scoreAt(gray.pixels, gray.width, gray.height, tmpl, ix + 1, iy);
            const double top = scoreAt(gray.pixels, gray.width, gray.height, tmpl, ix, iy - 1);
            const double bottom = scoreAt(gray.pixels, gray.width, gray.height, tmpl, ix, iy + 1);
            result.x = static_cast<double>(ix) + parabolicOffset(left, center, right);
            result.y = static_cast<double>(iy) + parabolicOffset(top, center, bottom);
            result.score = center;
        }
    }

    result.ok = result.score >= params.minScore;
    result.message = result.ok
        ? QStringLiteral("OK")
        : QStringLiteral("Best score below minScore");
    return result;
}

double GrayMatcher::scoreAt(const std::vector<double>& image,
                            int imageWidth,
                            int imageHeight,
                            const GrayTemplate& tmpl,
                            int x,
                            int y)
{
    if (x < 0 || y < 0 || x + tmpl.width > imageWidth || y + tmpl.height > imageHeight) {
        return -std::numeric_limits<double>::infinity();
    }

    double patchSum = 0.0;
    for (int ty = 0; ty < tmpl.height; ++ty) {
        const size_t rowOffset = static_cast<size_t>((y + ty) * imageWidth + x);
        for (int tx = 0; tx < tmpl.width; ++tx) {
            patchSum += image[rowOffset + static_cast<size_t>(tx)];
        }
    }

    const double count = static_cast<double>(tmpl.width * tmpl.height);
    const double patchMean = patchSum / count;
    double numerator = 0.0;
    double patchNormSq = 0.0;

    for (int ty = 0; ty < tmpl.height; ++ty) {
        const size_t imageRowOffset = static_cast<size_t>((y + ty) * imageWidth + x);
        const size_t tmplRowOffset = static_cast<size_t>(ty * tmpl.width);
        for (int tx = 0; tx < tmpl.width; ++tx) {
            const double imageCentered = image[imageRowOffset + static_cast<size_t>(tx)] - patchMean;
            const double templateCentered = tmpl.gray[tmplRowOffset + static_cast<size_t>(tx)] - tmpl.mean;
            numerator += imageCentered * templateCentered;
            patchNormSq += imageCentered * imageCentered;
        }
    }

    const double denominator = std::sqrt(patchNormSq) * tmpl.norm;
    if (denominator <= 1.0e-12) {
        return -std::numeric_limits<double>::infinity();
    }
    return numerator / denominator;
}

} // namespace VisionTools::Matching
