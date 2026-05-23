#include "VisionTools/FindCircleTool.h"
#include "VisionTools/ImageView.h"

#include <QCoreApplication>
#include <QImage>
#include <QtGlobal>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>

namespace {

constexpr double pi = 3.14159265358979323846;
constexpr double circleCenterX = 560.0;
constexpr double circleCenterY = 320.0;
constexpr double circleRadius = 96.0;

void quietQtMessages(QtMsgType, const QMessageLogContext&, const QString&)
{
}

QImage createSyntheticImage()
{
    constexpr int width = 800;
    constexpr int height = 600;
    QImage image(width, height, QImage::Format_Grayscale8);

    constexpr double lineCenterX = 260.0;
    constexpr double lineCenterY = 300.0;
    constexpr double lineAngleDeg = 8.0;
    const double lineAngleRad = lineAngleDeg * pi / 180.0;
    const double nx = -std::sin(lineAngleRad);
    const double ny = std::cos(lineAngleRad);

    std::mt19937 rng(42);
    std::uniform_int_distribution<int> noise(-4, 4);

    for (int y = 0; y < height; ++y) {
        uchar* row = image.scanLine(y);
        for (int x = 0; x < width; ++x) {
            const double lineDistance = (x - lineCenterX) * nx + (y - lineCenterY) * ny;
            int value = lineDistance < 0.0 ? 45 : 180;

            const double dx = x - circleCenterX;
            const double dy = y - circleCenterY;
            const double circleDistance = std::sqrt(dx * dx + dy * dy);
            if (circleDistance < circleRadius) {
                value = 55;
            }

            row[x] = static_cast<uchar>(std::clamp(value + noise(rng), 0, 255));
        }
    }

    return image;
}

} // namespace

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    qInstallMessageHandler(quietQtMessages);

    const QImage image = createSyntheticImage();

    VisionTools::CircleSearchRegion region;
    region.centerX = circleCenterX;
    region.centerY = circleCenterY;
    region.radius = circleRadius;
    region.searchLength = 120.0;
    region.caliperWidth = 48.0;
    region.startAngleDeg = 0.0;
    region.spanAngleDeg = 360.0;
    region.caliperCount = 48;
    region.searchDirection = VisionTools::CircleSearchDirection::Outward;

    VisionTools::FindCircleParams params;
    params.caliperParams.sampleCount = 161;
    params.caliperParams.projectionCount = 9;
    params.caliperParams.smoothingSigma = 1.0;
    params.caliperParams.minResponse = 5.0;
    params.caliperParams.polarity = VisionTools::EdgePolarity::DarkToLight;
    params.caliperParams.selection = VisionTools::EdgeSelection::Strongest;
    params.caliperParams.enableSubpixel = true;
    params.fitParams.maxResidual = 3.0;
    params.fitParams.minInlierCount = 12;
    params.fitParams.maxIterations = 20;
    params.fitParams.damping = 1.0e-3;
    params.fitParams.ransacResidual = 4.0;
    params.fitParams.huberDelta = 2.0;
    params.minEdgeResponse = params.caliperParams.minResponse;
    params.circleExpectedPositionTolerance = region.searchLength * 0.5;

    VisionTools::FindCircleTool tool;
    tool.setParams(params);

    int okCount = 0;
    bool stable = true;
    for (int i = 0; i < 20; ++i) {
        const VisionTools::FindCircleResult result = tool.run(VisionTools::ImageView(image), region);
        if (result.ok) {
            ++okCount;
        } else {
            stable = false;
        }
        if (result.diagnostics.fitInputPointsCount <= 0) {
            stable = false;
        }

        std::cout << "#" << (i + 1)
                  << " ok=" << (result.ok ? "true" : "false")
                  << " center=(" << result.fitResult.circle.centerX << "," << result.fitResult.circle.centerY << ")"
                  << " radius=" << result.fitResult.circle.radius
                  << " RMS=" << result.fitResult.rmsError
                  << " score=" << result.fitResult.score
                  << " accepted=" << result.diagnostics.fitInputPointsCount;
        if (!result.ok) {
            std::cout << " message=" << result.message.toStdString();
        }
        std::cout << std::endl;
    }

    std::cout << "summary okCount=" << okCount << "/20" << std::endl;
    return stable && okCount == 20 ? 0 : 1;
}
