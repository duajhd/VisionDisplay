#include "VisionTools/FindCircleTool.h"
#include "VisionTools/ImageView.h"

#include <QCoreApplication>
#include <QImage>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>

using namespace VisionTools;

namespace {

QImage createCircleEdgeImage(double centerX, double centerY, double radius)
{
    constexpr int width = 640;
    constexpr int height = 480;
    QImage image(width, height, QImage::Format_Grayscale8);

    std::mt19937 rng(11);
    std::uniform_int_distribution<int> noise(-3, 3);

    for (int y = 0; y < height; ++y) {
        uchar* row = image.scanLine(y);
        for (int x = 0; x < width; ++x) {
            const double dx = x - centerX;
            const double dy = y - centerY;
            const double distance = std::sqrt(dx * dx + dy * dy);
            const int base = distance < radius ? 50 : 200;
            row[x] = static_cast<uchar>(std::clamp(base + noise(rng), 0, 255));
        }
    }

    return image;
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    constexpr double trueCenterX = 318.0;
    constexpr double trueCenterY = 236.0;
    constexpr double trueRadius = 116.0;

    const QImage image = createCircleEdgeImage(trueCenterX, trueCenterY, trueRadius);
    const ImageView imageView(image);

    CircleSearchRegion region;
    region.centerX = 320.0;
    region.centerY = 240.0;
    region.radius = 115.0;
    region.searchLength = 50.0;
    region.caliperWidth = 12.0;
    region.startAngleDeg = 0.0;
    region.spanAngleDeg = 360.0;
    region.caliperCount = 32;
    region.searchDirection = CircleSearchDirection::Outward;

    FindCircleParams params;
    params.caliperParams.sampleCount = 81;
    params.caliperParams.projectionCount = 5;
    params.caliperParams.smoothingSigma = 1.0;
    params.caliperParams.minResponse = 5.0;
    params.caliperParams.polarity = EdgePolarity::DarkToLight;
    params.caliperParams.selection = EdgeSelection::Strongest;
    params.caliperParams.enableSubpixel = true;
    params.fitParams.maxResidual = 2.5;
    params.fitParams.minInlierCount = 12;
    params.fitParams.maxIterations = 20;
    params.fitParams.damping = 1.0e-3;
    params.minEdgeResponse = 5.0;

    FindCircleTool tool;
    tool.setParams(params);
    const FindCircleResult result = tool.run(imageView, region);

    std::cout << "FindCircle ok: " << (result.ok ? "true" : "false") << "\n";
    std::cout << "message: " << result.message.toStdString() << "\n";
    std::cout << "calipers: " << result.calipers.size() << "\n";
    std::cout << "edge points: " << result.edgePoints.size() << "\n";
    std::cout << "inliers: " << result.fitResult.inlierPoints.size() << "\n";
    std::cout << "outliers: " << result.fitResult.outlierPoints.size() << "\n";
    std::cout << "true circle: cx=" << trueCenterX
              << ", cy=" << trueCenterY
              << ", r=" << trueRadius << "\n";
    if (result.ok) {
        std::cout << "fitted circle: cx=" << result.fitResult.circle.centerX
                  << ", cy=" << result.fitResult.circle.centerY
                  << ", r=" << result.fitResult.circle.radius << "\n";
        std::cout << "rms error: " << result.fitResult.rmsError << "\n";
        std::cout << "max error: " << result.fitResult.maxError << "\n";
        std::cout << "score: " << result.fitResult.score << "\n";
    }

    return result.ok ? 0 : 1;
}
