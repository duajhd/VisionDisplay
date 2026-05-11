#include "VisionTools/FindRectTool.h"
#include "VisionTools/ImageView.h"

#include <QCoreApplication>
#include <QImage>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>

using namespace VisionTools;

namespace {

constexpr double pi = 3.14159265358979323846;

QImage createRectEdgeImage(double centerX, double centerY, double width, double height, double angleDeg)
{
    constexpr int imageWidth = 760;
    constexpr int imageHeight = 560;
    QImage image(imageWidth, imageHeight, QImage::Format_Grayscale8);

    const double angleRad = angleDeg * pi / 180.0;
    const double ux = std::cos(angleRad);
    const double uy = std::sin(angleRad);
    const double vx = -std::sin(angleRad);
    const double vy = std::cos(angleRad);

    std::mt19937 rng(23);
    std::uniform_int_distribution<int> noise(-4, 4);

    for (int y = 0; y < imageHeight; ++y) {
        uchar* row = image.scanLine(y);
        for (int x = 0; x < imageWidth; ++x) {
            const double dx = x - centerX;
            const double dy = y - centerY;
            const double localX = dx * ux + dy * uy;
            const double localY = dx * vx + dy * vy;
            int base = (std::abs(localX) <= width * 0.5 && std::abs(localY) <= height * 0.5) ? 55 : 200;

            const bool topGap = std::abs(localY + height * 0.5) < 3.0 && localX > -95.0 && localX < -50.0;
            const bool rightGap = std::abs(localX - width * 0.5) < 3.0 && localY > 25.0 && localY < 70.0;
            if (topGap || rightGap) {
                base = 200;
            }

            const bool distractor = std::abs(localX - width * 0.5 - 18.0) < 2.0 && localY > -90.0 && localY < -40.0;
            if (distractor) {
                base = 55;
            }

            row[x] = static_cast<uchar>(std::clamp(base + noise(rng), 0, 255));
        }
    }

    return image;
}

const FindRectEdgeResult& edgeResult(const FindRectResult& result, RectEdgeId edgeId)
{
    switch (edgeId) {
    case RectEdgeId::Top: return result.top;
    case RectEdgeId::Bottom: return result.bottom;
    case RectEdgeId::Left: return result.left;
    case RectEdgeId::Right: return result.right;
    }
    return result.top;
}

const char* edgeName(RectEdgeId edgeId)
{
    switch (edgeId) {
    case RectEdgeId::Top: return "top";
    case RectEdgeId::Bottom: return "bottom";
    case RectEdgeId::Left: return "left";
    case RectEdgeId::Right: return "right";
    }
    return "edge";
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    constexpr double trueCenterX = 382.0;
    constexpr double trueCenterY = 276.0;
    constexpr double trueWidth = 310.0;
    constexpr double trueHeight = 185.0;
    constexpr double trueAngleDeg = 10.0;

    const QImage image = createRectEdgeImage(trueCenterX, trueCenterY, trueWidth, trueHeight, trueAngleDeg);
    const ImageView imageView(image);

    FindRectParams params;
    params.centerX = 380.0;
    params.centerY = 278.0;
    params.expectedWidth = 305.0;
    params.expectedHeight = 188.0;
    params.angleDeg = 9.0;
    params.calipersPerEdge = 21;
    params.searchLength = 56.0;
    params.projectionWidth = 12.0;
    params.polarity = EdgePolarity::LightToDark;
    params.selection = EdgeSelection::Strongest;
    params.minResponse = 5.0;
    params.maxLineResidual = 2.5;
    params.maxParallelAngleErrorDeg = 3.0;
    params.maxOrthogonalAngleErrorDeg = 3.0;
    params.maxSizeErrorRatio = 0.18;
    params.searchDirection = RectSearchDirection::FromOutsideToInside;

    FindRectTool tool;
    tool.setParams(params);
    const FindRectResult result = tool.run(imageView);

    std::cout << "FindRect ok: " << (result.ok ? "true" : "false") << "\n";
    std::cout << "message: " << result.message.toStdString() << "\n";
    std::cout << "true rect: cx=" << trueCenterX
              << ", cy=" << trueCenterY
              << ", w=" << trueWidth
              << ", h=" << trueHeight
              << ", angle=" << trueAngleDeg << "\n";

    if (result.ok) {
        std::cout << "fitted rect: cx=" << result.rect.centerX
                  << ", cy=" << result.rect.centerY
                  << ", w=" << result.rect.width
                  << ", h=" << result.rect.height
                  << ", angle=" << result.rect.angleDeg << "\n";
        std::cout << "rms error: " << result.rmsError << "\n";
        std::cout << "max error: " << result.maxError << "\n";
        std::cout << "score: " << result.score << "\n";
    }

    for (RectEdgeId edgeId : {RectEdgeId::Top, RectEdgeId::Bottom, RectEdgeId::Left, RectEdgeId::Right}) {
        const FindRectEdgeResult& edge = edgeResult(result, edgeId);
        std::cout << edgeName(edgeId)
                  << ": ok=" << (edge.ok ? "true" : "false")
                  << ", input=" << edge.inputPoints.size()
                  << ", inliers=" << edge.inlierPoints.size()
                  << ", outliers=" << edge.outlierPoints.size()
                  << ", rms=" << edge.rmsError
                  << ", score=" << edge.score
                  << ", message=" << edge.message.toStdString()
                  << "\n";
    }

    return result.ok ? 0 : 1;
}
