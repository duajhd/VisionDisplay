#include "VisionTools/FindEllipseTool.h"
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

QImage createEllipseImage(double centerX, double centerY, double radiusA, double radiusB, double angleDeg)
{
    constexpr int width = 800;
    constexpr int height = 600;
    QImage image(width, height, QImage::Format_Grayscale8);

    const double theta = angleDeg * pi / 180.0;
    const double c = std::cos(theta);
    const double s = std::sin(theta);

    std::mt19937 rng(31);
    std::uniform_int_distribution<int> noise(-4, 4);

    for (int y = 0; y < height; ++y) {
        uchar* row = image.scanLine(y);
        for (int x = 0; x < width; ++x) {
            const double dx = x - centerX;
            const double dy = y - centerY;
            const double xLocal = c * dx + s * dy;
            const double yLocal = -s * dx + c * dy;
            const double value = xLocal * xLocal / (radiusA * radiusA)
                               + yLocal * yLocal / (radiusB * radiusB);
            int base = value <= 1.0 ? 200 : 50;

            const bool localGap = value > 0.96 && value < 1.05 && xLocal > -80.0 && xLocal < -35.0 && yLocal > 55.0;
            if (localGap) {
                base = 50;
            }

            const bool distractor = value > 1.15 && value < 1.22 && xLocal > 60.0 && xLocal < 120.0 && yLocal < -20.0;
            if (distractor) {
                base = 200;
            }

            const bool strongOutlierEdge = std::abs(xLocal - radiusA - 22.0) < 2.5 && yLocal > -60.0 && yLocal < 55.0;
            if (strongOutlierEdge) {
                base = 200;
            }

            row[x] = static_cast<uchar>(std::clamp(base + noise(rng), 0, 255));
        }
    }

    return image;
}

const char* robustLossName(RobustLossType lossType)
{
    switch (lossType) {
    case RobustLossType::None: return "None";
    case RobustLossType::Huber: return "Huber";
    case RobustLossType::Tukey: return "Tukey";
    }
    return "Unknown";
}

FindEllipseResult runFindEllipse(const ImageView& imageView, RobustLossType lossType)
{
    FindEllipseParams params;
    params.centerX = 398.0;
    params.centerY = 302.0;
    params.radiusA = 176.0;
    params.radiusB = 92.0;
    params.angleDeg = 23.0;
    params.startAngleDeg = 0.0;
    params.spanAngleDeg = 360.0;
    params.caliperCount = 48;
    params.searchLength = 75.0;
    params.projectionWidth = 12.0;
    params.polarity = EdgePolarity::DarkToLight;
    params.selection = EdgeSelection::Strongest;
    params.minResponse = 5.0;
    params.maxResidual = 3.0;
    params.maxIterations = 35;
    params.searchDirection = EllipseSearchDirection::FromOutsideToInside;
    params.enableRobustFiltering = false;
    params.robust.lossType = lossType;
    params.robust.huberDelta = 2.0;
    params.robust.tukeyC = 4.685;
    params.robust.irlsIterations = 12;
    params.robust.minWeight = 1.0e-6;

    FindEllipseTool tool;
    tool.setParams(params);
    return tool.run(imageView);
}

void printResult(RobustLossType lossType, const FindEllipseResult& result)
{
    std::cout << "\n=== Robust loss: " << robustLossName(lossType) << " ===\n";
    std::cout << "FindEllipse ok: " << (result.ok ? "true" : "false") << "\n";
    std::cout << "message: " << result.message.toStdString() << "\n";
    std::cout << "calipers: " << result.calipers.size() << "\n";
    std::cout << "input points: " << result.inputPoints.size() << "\n";
    std::cout << "inliers: " << result.inlierPoints.size() << "\n";
    std::cout << "outliers: " << result.outlierPoints.size() << "\n";
    std::cout << "fitted ellipse: cx=" << result.fittedEllipse.centerX
              << ", cy=" << result.fittedEllipse.centerY
              << ", a=" << result.fittedEllipse.radiusA
              << ", b=" << result.fittedEllipse.radiusB
              << ", angle=" << result.fittedEllipse.angleDeg << "\n";
    std::cout << "rms error: " << result.rmsError << "\n";
    std::cout << "max error: " << result.maxError << "\n";
    std::cout << "score: " << result.score << "\n";
    std::cout << "diagnostics: inlierRatio=" << result.diagnostics.inlierRatio
              << ", medianError=" << result.diagnostics.medianError
              << ", meanAbsError=" << result.diagnostics.meanAbsError
              << ", angleErrorDeg=" << result.diagnostics.angleErrorDeg << "\n";

    if (!result.pointResiduals.empty() && result.pointResiduals.size() == result.pointWeights.size()) {
        double minWeight = 1.0;
        double maxAbsResidual = 0.0;
        for (size_t i = 0; i < result.pointResiduals.size(); ++i) {
            minWeight = std::min(minWeight, result.pointWeights[i]);
            maxAbsResidual = std::max(maxAbsResidual, std::abs(result.pointResiduals[i]));
        }
        std::cout << "point debug: residualCount=" << result.pointResiduals.size()
                  << ", maxAbsResidual=" << maxAbsResidual
                  << ", minWeight=" << minWeight << "\n";
    }
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    constexpr double trueCenterX = 400.0;
    constexpr double trueCenterY = 300.0;
    constexpr double trueRadiusA = 180.0;
    constexpr double trueRadiusB = 90.0;
    constexpr double trueAngleDeg = 25.0;

    const QImage image = createEllipseImage(trueCenterX, trueCenterY, trueRadiusA, trueRadiusB, trueAngleDeg);
    const ImageView imageView(image);

    std::cout << "true ellipse: cx=" << trueCenterX
              << ", cy=" << trueCenterY
              << ", a=" << trueRadiusA
              << ", b=" << trueRadiusB
              << ", angle=" << trueAngleDeg << "\n";

    const FindEllipseResult none = runFindEllipse(imageView, RobustLossType::None);
    const FindEllipseResult huber = runFindEllipse(imageView, RobustLossType::Huber);
    const FindEllipseResult tukey = runFindEllipse(imageView, RobustLossType::Tukey);

    printResult(RobustLossType::None, none);
    printResult(RobustLossType::Huber, huber);
    printResult(RobustLossType::Tukey, tukey);

    return (none.ok && huber.ok && tukey.ok) ? 0 : 1;
}
