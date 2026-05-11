#pragma once

#include <QString>
#include <QVariantList>

#include <vector>

namespace VisionDisplay {
class VisionDisplayItem;
}

namespace VisionTools {
struct CaliperRegion;
struct CaliperResult;
struct CircleSearchRegion;
struct EdgePoint;
struct FindCircleResult;
struct FindEllipseResult;
struct FindLineResult;
struct LineSearchRegion;
}

class ToolResultDisplayAdapter
{
public:
    static void showCaliper(VisionDisplay::VisionDisplayItem* display,
                            const QString& toolId,
                            const VisionTools::CaliperRegion& region,
                            const VisionTools::CaliperResult& result);

    static void showFindLine(VisionDisplay::VisionDisplayItem* display,
                             const QString& toolId,
                             const VisionTools::LineSearchRegion& region,
                             const VisionTools::FindLineResult& result);

    static void showFindCircle(VisionDisplay::VisionDisplayItem* display,
                               const QString& toolId,
                               const VisionTools::CircleSearchRegion& region,
                               const VisionTools::FindCircleResult& result);

    static void showFindEllipse(VisionDisplay::VisionDisplayItem* display,
                                const QString& toolId,
                                const VisionTools::FindEllipseResult& result);

private:
    static QVariantList edgePointsToVariantList(const std::vector<VisionTools::EdgePoint>& points, bool outlier = false);
    static int selectedEdgeIndex(const std::vector<VisionTools::EdgePoint>& candidates,
                                 const std::vector<VisionTools::EdgePoint>& selectedEdges);
};
