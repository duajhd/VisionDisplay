#include "VisionTools/CaliperTool.h"
#include "VisionTools/FindCircleTool.h"
#include "VisionTools/ImageView.h"

#include <QImage>

#include <cmath>
#include <iostream>

namespace {

QImage createStepImage(int edgeX)
{
    QImage image(160, 80, QImage::Format_Grayscale8);
    for (int y = 0; y < image.height(); ++y) {
        uchar* row = image.scanLine(y);
        for (int x = 0; x < image.width(); ++x) {
            row[x] = x < edgeX ? 50 : 180;
        }
    }
    return image;
}

bool runExpectedSelectionCase(double projectionWidth, int projectionCount)
{
    const QImage image = createStepImage(80);

    VisionTools::CaliperRegion region;
    region.centerX = 80.0;
    region.centerY = 40.0;
    region.length = 80.0;
    region.width = projectionWidth;
    region.angleDeg = 0.0;

    VisionTools::CaliperParams params;
    params.sampleCount = 161;
    params.projectionCount = projectionCount;
    params.smoothingSigma = 1.0;
    params.minResponse = 5.0;
    params.polarity = VisionTools::EdgePolarity::DarkToLight;
    params.selection = VisionTools::EdgeSelection::NearestToExpected;
    params.expectedPosition1D = 0.0;
    params.maxPositionDeviation = 10.0;
    params.allowFallbackSelection = false;
    params.enableSubpixel = true;

    VisionTools::CaliperTool tool;
    tool.setParams(params);
    const VisionTools::CaliperResult result = tool.run(VisionTools::ImageView(image), region);
    if (!result.ok || result.selectedEdges.empty()) {
        std::cerr << "Expected selection failed for width=" << projectionWidth
                  << " projectionCount=" << projectionCount << "\n";
        return false;
    }

    const double position = result.selectedEdges.front().position1D;
    const bool withinExpected = std::abs(position - params.expectedPosition1D) <= params.maxPositionDeviation;
    std::cout << "width=" << projectionWidth
              << " projectionCount=" << projectionCount
              << " selectedPosition1D=" << position
              << " withinExpected=" << (withinExpected ? "true" : "false") << std::endl;
    return withinExpected && std::abs(position) <= 2.0 && !result.selectedByFallback;
}

bool runNoFallbackCase()
{
    const QImage image = createStepImage(92);

    VisionTools::CaliperRegion region;
    region.centerX = 80.0;
    region.centerY = 40.0;
    region.length = 80.0;
    region.width = 1.0;
    region.angleDeg = 0.0;

    VisionTools::CaliperParams params;
    params.sampleCount = 161;
    params.projectionCount = 1;
    params.smoothingSigma = 1.0;
    params.minResponse = 5.0;
    params.polarity = VisionTools::EdgePolarity::DarkToLight;
    params.selection = VisionTools::EdgeSelection::NearestToExpected;
    params.fallbackSelection = VisionTools::EdgeSelection::Strongest;
    params.expectedPosition1D = 0.0;
    params.maxPositionDeviation = 10.0;
    params.allowFallbackSelection = false;
    params.enableSubpixel = true;

    VisionTools::CaliperTool tool;
    tool.setParams(params);
    const VisionTools::CaliperResult result = tool.run(VisionTools::ImageView(image), region);
    std::cout << "outsideWindow candidates=" << result.candidates.size()
              << " ok=" << (result.ok ? "true" : "false")
              << " selectedByFallback=" << (result.selectedByFallback ? "true" : "false") << std::endl;
    return !result.candidates.empty() && !result.ok && result.selectedEdges.empty() && !result.selectedByFallback;
}

QImage createCircleImage()
{
    QImage image(220, 220, QImage::Format_Grayscale8);
    const double cx = 110.0;
    const double cy = 110.0;
    for (int y = 0; y < image.height(); ++y) {
        uchar* row = image.scanLine(y);
        for (int x = 0; x < image.width(); ++x) {
            const double dx = static_cast<double>(x) - cx;
            const double dy = static_cast<double>(y) - cy;
            const double r = std::sqrt(dx * dx + dy * dy);
            row[x] = r < 50.0 ? 50 : (r < 62.0 ? 95 : 240);
        }
    }
    return image;
}

bool runFindCircleExpectedWindowCase()
{
    const QImage image = createCircleImage();

    VisionTools::CircleSearchRegion region;
    region.centerX = 110.0;
    region.centerY = 110.0;
    region.radius = 50.0;
    region.searchLength = 40.0;
    region.caliperWidth = 1.0;
    region.caliperCount = 12;
    region.searchDirection = VisionTools::CircleSearchDirection::Outward;

    VisionTools::FindCircleParams params;
    params.caliperParams.sampleCount = 161;
    params.caliperParams.projectionCount = 1;
    params.caliperParams.smoothingSigma = 1.0;
    params.caliperParams.minResponse = 5.0;
    params.caliperParams.polarity = VisionTools::EdgePolarity::DarkToLight;
    params.caliperParams.selection = VisionTools::EdgeSelection::NearestToExpected;
    params.caliperParams.expectedPosition1D = 0.0;
    params.caliperParams.maxPositionDeviation = 10.0;
    params.caliperParams.allowFallbackSelection = false;
    params.fitParams.maxResidual = 3.0;
    params.fitParams.minInlierCount = 6;
    params.fitParams.enableRansac = false;
    params.fitParams.maxIterations = 5;
    params.minEdgeResponse = 5.0;

    VisionTools::FindCircleTool tool;
    tool.setParams(params);
    const VisionTools::FindCircleResult result = tool.run(VisionTools::ImageView(image), region);
    const double radiusError = std::abs(result.fitResult.circle.radius - region.radius);
    std::cout << "findCircle ok=" << (result.ok ? "true" : "false")
              << " edgePoints=" << result.edgePoints.size()
              << " rejected=" << result.rejectedEdgePoints.size()
              << " outliers=" << result.outlierPoints.size()
              << " radius=" << result.fitResult.circle.radius
              << " radiusError=" << radiusError << std::endl;
    return result.ok && radiusError <= 1.5 && !result.edgePoints.empty();
}

} // namespace

int main()
{
    bool ok = true;
    std::cout << "running width=1 projectionCount=1" << std::endl;
    ok = runExpectedSelectionCase(1.0, 1) && ok;
    std::cout << "running width=48 projectionCount=9" << std::endl;
    ok = runExpectedSelectionCase(48.0, 9) && ok;
    std::cout << "running no fallback case" << std::endl;
    ok = runNoFallbackCase() && ok;
    std::cout << "running find circle expected window case" << std::endl;
    ok = runFindCircleExpectedWindowCase() && ok;
    return ok ? 0 : 1;
}
