#include "VisionTools/FindLineTool.h"
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

QImage createSlantedEdgeImage(double lineAngleDeg)
{
    constexpr int width = 640;
    constexpr int height = 480;
    QImage image(width, height, QImage::Format_Grayscale8);

    const double angleRad = lineAngleDeg * pi / 180.0;
    const double nx = -std::sin(angleRad);
    const double ny = std::cos(angleRad);
    const double cx = width * 0.5;
    const double cy = height * 0.5;

    std::mt19937 rng(7);
    std::uniform_int_distribution<int> noise(-3, 3);

    for (int y = 0; y < height; ++y) {
        uchar* row = image.scanLine(y);
        for (int x = 0; x < width; ++x) {
            const double signedDistance = (x - cx) * nx + (y - cy) * ny;
            const int base = signedDistance < 0.0 ? 50 : 200;
            row[x] = static_cast<uchar>(std::clamp(base + noise(rng), 0, 255));
        }
    }

    return image;
}

double fittedLineAngleDeg(const LineModel& line)
{
    const double angle = std::atan2(line.y2 - line.y1, line.x2 - line.x1) * 180.0 / pi;
    return angle < 0.0 ? angle + 180.0 : angle;
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    constexpr double trueAngleDeg = 12.0;
    const QImage image = createSlantedEdgeImage(trueAngleDeg);
    const ImageView imageView(image);

    LineSearchRegion region;
    region.centerX = 320.0;
    region.centerY = 240.0;
    region.length = 360.0;
    region.searchLength = 100.0;
    region.angleDeg = trueAngleDeg;
    region.caliperCount = 15;
    region.caliperWidth = 18.0;

    FindLineParams params;
    params.caliperParams.sampleCount = 101;
    params.caliperParams.projectionCount = 7;
    params.caliperParams.smoothingSigma = 1.0;
    params.caliperParams.minResponse = 5.0;
    params.caliperParams.polarity = EdgePolarity::DarkToLight;
    params.caliperParams.selection = EdgeSelection::Strongest;
    params.caliperParams.enableSubpixel = true;
    params.fitParams.maxResidual = 2.0;
    params.fitParams.minInlierCount = 6;
    params.minEdgeResponse = 5.0;

    FindLineTool tool;
    tool.setParams(params);
    const FindLineResult result = tool.run(imageView, region);

    std::cout << "FindLine ok: " << (result.ok ? "true" : "false") << "\n";
    std::cout << "message: " << result.message.toStdString() << "\n";
    std::cout << "calipers: " << result.calipers.size() << "\n";
    std::cout << "edge points: " << result.edgePoints.size() << "\n";
    std::cout << "inliers: " << result.fitResult.inlierPoints.size() << "\n";
    std::cout << "outliers: " << result.fitResult.outlierPoints.size() << "\n";
    std::cout << "true angle deg: " << trueAngleDeg << "\n";
    if (result.ok) {
        std::cout << "fitted angle deg: " << fittedLineAngleDeg(result.fitResult.line) << "\n";
        std::cout << "rms error: " << result.fitResult.rmsError << "\n";
        std::cout << "max error: " << result.fitResult.maxError << "\n";
        std::cout << "score: " << result.fitResult.score << "\n";
        std::cout << "line endpoints: ("
                  << result.fitResult.line.x1 << ", " << result.fitResult.line.y1 << ") -> ("
                  << result.fitResult.line.x2 << ", " << result.fitResult.line.y2 << ")\n";
    }

    return result.ok ? 0 : 1;
}
