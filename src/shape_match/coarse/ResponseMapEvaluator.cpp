#include "shape_match/coarse/ResponseMapEvaluator.h"

#include "shape_match/core/ShapeMatchTypes.h"

#include <algorithm>
#include <cmath>

namespace ShapeMatch {

namespace {

int wrapBin(int bin, int count)
{
    if (count <= 0) {
        return 0;
    }
    bin %= count;
    return bin < 0 ? bin + count : bin;
}

int orientationBinFromVector(double x, double y, int binCount)
{
    const double angle = std::atan2(y, x);
    const double normalized = (angle + kPi) / (2.0 * kPi);
    int bin = static_cast<int>(std::floor(normalized * static_cast<double>(binCount)));
    return wrapBin(bin, binCount);
}

} // namespace

cv::Mat ResponseMapEvaluator::evaluateTheta(const OrientationResponseMap& responseMap,
                                            const ResponseTemplate& responseTemplate,
                                            int,
                                            double thetaRad,
                                            const OrientationResponseConfig&) const
{
    cv::Mat responseXY = cv::Mat::zeros(responseMap.imageSize, CV_32F);
    if (responseMap.empty() || responseTemplate.empty()) {
        return responseXY;
    }

    const double c = std::cos(thetaRad);
    const double s = std::sin(thetaRad);
    const int width = responseMap.imageSize.width;
    const int height = responseMap.imageSize.height;

    for (const ResponseTemplatePoint& p : responseTemplate.points) {
        const int ox = static_cast<int>(std::round(c * p.x - s * p.y));
        const int oy = static_cast<int>(std::round(s * p.x + c * p.y));
        const double rgx = c * p.gx - s * p.gy;
        const double rgy = s * p.gx + c * p.gy;
        const int bin = orientationBinFromVector(rgx, rgy, responseMap.orientationBinCount);
        const cv::Mat& binMap = responseMap.binMaps[static_cast<size_t>(bin)];

        const int cx0 = std::max(0, -ox);
        const int cy0 = std::max(0, -oy);
        const int cx1 = std::min(width, width - ox);
        const int cy1 = std::min(height, height - oy);
        if (cx0 >= cx1 || cy0 >= cy1) {
            continue;
        }

        for (int cy = cy0; cy < cy1; ++cy) {
            const float* src = binMap.ptr<float>(cy + oy) + (cx0 + ox);
            float* dst = responseXY.ptr<float>(cy) + cx0;
            for (int cx = cx0; cx < cx1; ++cx) {
                *dst += (*src) * p.weight;
                ++dst;
                ++src;
            }
        }
    }

    if (!responseTemplate.points.empty()) {
        responseXY *= 1.0f / static_cast<float>(responseTemplate.points.size());
    }
    return responseXY;
}

} // namespace ShapeMatch
