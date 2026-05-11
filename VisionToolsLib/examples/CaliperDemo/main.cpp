#include "VisionTools/CaliperTool.h"
#include "VisionTools/ImageView.h"

#include <QCoreApplication>
#include <QImage>

#include <algorithm>
#include <iostream>
#include <random>
#include <string>

using namespace VisionTools;

namespace {

QImage createSyntheticImage()
{
    constexpr int width = 640;
    constexpr int height = 480;
    QImage image(width, height, QImage::Format_Grayscale8);

    std::mt19937 rng(42);
    std::uniform_int_distribution<int> noise(-3, 3);

    for (int y = 0; y < height; ++y) {
        uchar* row = image.scanLine(y);
        for (int x = 0; x < width; ++x) {
            const int base = x < 320 ? 50 : 200;
            row[x] = static_cast<uchar>(std::clamp(base + noise(rng), 0, 255));
        }
    }

    return image;
}

std::string polarityName(EdgePolarity polarity)
{
    switch (polarity) {
    case EdgePolarity::DarkToLight:
        return "DarkToLight";
    case EdgePolarity::LightToDark:
        return "LightToDark";
    case EdgePolarity::Any:
        return "Any";
    }
    return "Unknown";
}

void printProfileWindow(const CaliperResult& result)
{
    if (result.profile.empty()) {
        return;
    }

    const int center = static_cast<int>(result.profile.size() / 2);
    const int first = std::max(0, center - 3);
    const int last = std::min(static_cast<int>(result.profile.size()) - 1, center + 3);

    std::cout << "  profile[" << first << ".." << last << "] =";
    for (int i = first; i <= last; ++i) {
        std::cout << ' ' << result.profile[static_cast<size_t>(i)];
    }
    std::cout << '\n';
}

void runCase(const ImageView& image, EdgePolarity polarity)
{
    CaliperRegion region;
    region.centerX = 320.0;
    region.centerY = 240.0;
    region.length = 160.0;
    region.width = 40.0;
    region.angleDeg = 0.0;

    CaliperParams params;
    params.sampleCount = 161;
    params.projectionCount = 11;
    params.smoothingSigma = 1.0;
    params.minResponse = 5.0;
    params.polarity = polarity;
    params.selection = EdgeSelection::Strongest;
    params.enableSubpixel = true;

    CaliperTool tool;
    tool.setParams(params);
    const CaliperResult result = tool.run(image, region);

    std::cout << polarityName(polarity) << "\n";
    std::cout << "  ok: " << (result.ok ? "true" : "false") << "\n";
    std::cout << "  message: " << result.message.toStdString() << "\n";
    std::cout << "  candidates: " << result.candidates.size() << "\n";

    if (!result.selectedEdges.empty()) {
        const EdgePoint& edge = result.selectedEdges.front();
        std::cout << "  selected edge: x=" << edge.x
                  << ", y=" << edge.y
                  << ", response=" << edge.response
                  << ", position1D=" << edge.position1D
                  << ", subIndex=" << edge.subIndex << "\n";
    }

    printProfileWindow(result);
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    const QImage image = createSyntheticImage();
    const ImageView imageView(image);

    runCase(imageView, EdgePolarity::DarkToLight);
    runCase(imageView, EdgePolarity::LightToDark);
    runCase(imageView, EdgePolarity::Any);

    return 0;
}
