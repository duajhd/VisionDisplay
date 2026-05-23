#include "ShapeModelOverlayBuilder.h"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace {

QPointF pointPos(const VisionTools::Matching::TemplatePoint& point)
{
    return QPointF(point.imagePos.x(), point.imagePos.y());
}

QPointF vectorPoint(const Eigen::Vector2d& v, double length)
{
    return QPointF(v.x() * length, v.y() * length);
}

QColor chainColor(int id)
{
    static const QColor colors[] = {
        QColor(0, 220, 120), QColor(80, 170, 255), QColor(255, 210, 60),
        QColor(255, 90, 120), QColor(190, 120, 255), QColor(80, 230, 230),
        QColor(255, 150, 60), QColor(170, 230, 80)
    };
    return colors[static_cast<size_t>(std::abs(id)) % std::size(colors)];
}

void addPoint(VisionDisplay::VisionDisplayOverlayData* data,
              const VisionTools::Matching::TemplatePoint& point,
              const QColor& color,
              double radius,
              const QString& label = {})
{
    VisionDisplay::OverlayPoint overlayPoint;
    overlayPoint.pos = pointPos(point);
    overlayPoint.color = color;
    overlayPoint.radius = radius;
    overlayPoint.label = label;
    data->points.push_back(overlayPoint);
}

} // namespace

VisionDisplay::VisionDisplayOverlayData ShapeModelOverlayBuilder::build(
    const VisionTools::Matching::ShapeTemplateModel& model,
    int level,
    const ShapeModelOverlayOptions& options)
{
    VisionDisplay::VisionDisplayOverlayData data;
    if (level < 0 || level >= static_cast<int>(model.levels.size())) {
        return data;
    }

    const VisionTools::Matching::TemplateLevel& templateLevel = model.levels[static_cast<size_t>(level)];

    if (options.showChains) {
        for (const VisionTools::Matching::EdgeChain& chain : templateLevel.chains) {
            if (chain.points.size() < 2) {
                continue;
            }
            VisionDisplay::OverlayPolyline polyline;
            polyline.color = chainColor(chain.id);
            polyline.width = 1.4;
            polyline.points.reserve(static_cast<int>(chain.points.size()));
            for (const VisionTools::Matching::TemplatePoint& point : chain.points) {
                polyline.points.push_back(pointPos(point));
            }
            data.polylines.push_back(polyline);

            if (options.showChainId) {
                const VisionTools::Matching::TemplatePoint& mid = chain.points[chain.points.size() / 2];
                VisionDisplay::OverlayText text;
                text.pos = pointPos(mid);
                text.text = QStringLiteral("C%1").arg(chain.id);
                text.color = polyline.color;
                text.fontSize = 12;
                data.texts.push_back(text);
            }
        }
    }

    if (options.showAllEdges) {
        for (const VisionTools::Matching::TemplatePoint& point : templateLevel.allEdgePoints) {
            addPoint(&data, point, QColor(130, 130, 130), 1.4);
        }
    }

    if (options.showStablePoints) {
        for (const VisionTools::Matching::TemplatePoint& point : templateLevel.stablePoints) {
            addPoint(&data, point, QColor(30, 230, 90), 1.7);
        }
    }

    if (options.showRejectedPoints) {
        for (const VisionTools::Matching::TemplatePoint& point : templateLevel.rejectedPoints) {
            addPoint(&data, point, QColor(255, 70, 70), 2.0);
        }
    }

    if (options.showSampledPoints) {
        int labelCount = 0;
        for (const VisionTools::Matching::TemplatePoint& point : templateLevel.points) {
            const QString label = options.showPointId && labelCount < 80
                ? QString::number(point.id)
                : QString();
            addPoint(&data, point, QColor(255, 220, 50), 2.4, label);
            ++labelCount;
        }
    }

    if (options.showCurvature) {
        for (const VisionTools::Matching::TemplatePoint& point : templateLevel.points) {
            if (!point.isCorner) {
                continue;
            }
            addPoint(&data, point, QColor(220, 90, 255), 3.3);
        }
    }

    if (options.showNormals) {
        const int step = std::max(1, options.normalStep);
        for (int i = 0; i < static_cast<int>(templateLevel.points.size()); i += step) {
            const VisionTools::Matching::TemplatePoint& point = templateLevel.points[static_cast<size_t>(i)];
            VisionDisplay::OverlayArrow arrow;
            arrow.p0 = pointPos(point);
            arrow.p1 = arrow.p0 + vectorPoint(point.normal, options.normalLength);
            arrow.color = QColor(80, 160, 255);
            arrow.width = 1.2;
            data.arrows.push_back(arrow);
        }
    }

    if (options.showTangents) {
        const int step = std::max(1, options.tangentStep);
        for (int i = 0; i < static_cast<int>(templateLevel.points.size()); i += step) {
            const VisionTools::Matching::TemplatePoint& point = templateLevel.points[static_cast<size_t>(i)];
            VisionDisplay::OverlayArrow arrow;
            arrow.p0 = pointPos(point) - vectorPoint(point.tangent, options.tangentLength * 0.5);
            arrow.p1 = pointPos(point) + vectorPoint(point.tangent, options.tangentLength * 0.5);
            arrow.color = QColor(60, 230, 230);
            arrow.width = 1.2;
            data.arrows.push_back(arrow);
        }
    }

    return data;
}
