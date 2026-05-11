#include "VisionDisplay/RoiObject.h"

#include <QJsonArray>
#include <QLineF>
#include <QPolygonF>
#include <QTransform>

#include <algorithm>
#include <cmath>

namespace VisionDisplay {

namespace {

constexpr int circleSegments = 96;
constexpr double pi = 3.14159265358979323846;

double pointLineDistance(const QPointF& point, const QLineF& line)
{
    const QPointF a = line.p1();
    const QPointF b = line.p2();
    const QPointF ab = b - a;
    const QPointF ap = point - a;
    const double lengthSquared = QPointF::dotProduct(ab, ab);
    if (lengthSquared <= 0.0) {
        return QLineF(point, a).length();
    }

    const double t = std::clamp(QPointF::dotProduct(ap, ab) / lengthSquared, 0.0, 1.0);
    const QPointF projection = a + ab * t;
    return QLineF(point, projection).length();
}

QJsonArray pointArray(const QPointF& point)
{
    QJsonArray array;
    array.append(point.x());
    array.append(point.y());
    return array;
}

QJsonObject baseRoiJson(const RoiObject& roi, const QString& type)
{
    return {
        {QStringLiteral("id"), roi.id},
        {QStringLiteral("type"), type},
        {QStringLiteral("visible"), roi.visible},
        {QStringLiteral("locked"), roi.locked},
        {QStringLiteral("style"), QJsonObject{
            {QStringLiteral("strokeColor"), roi.strokeColor.name(QColor::HexArgb)},
            {QStringLiteral("selectedStrokeColor"), roi.selectedStrokeColor.name(QColor::HexArgb)}
        }}
    };
}

QVector<QPointF> rotatedRectPoints(const QPointF& center, double width, double height, double angleDeg)
{
    const double halfWidth = width * 0.5;
    const double halfHeight = height * 0.5;
    QVector<QPointF> localPoints{
        QPointF(-halfWidth, -halfHeight),
        QPointF(halfWidth, -halfHeight),
        QPointF(halfWidth, halfHeight),
        QPointF(-halfWidth, halfHeight),
        QPointF(-halfWidth, -halfHeight)
    };

    QTransform transform;
    transform.translate(center.x(), center.y());
    transform.rotate(angleDeg);

    QVector<QPointF> points;
    points.reserve(localPoints.size());
    for (const QPointF& point : localPoints) {
        points.push_back(transform.map(point));
    }
    return points;
}

QPointF rotateIntoLocal(const QPointF& point, const QPointF& center, double angleDeg)
{
    QTransform transform;
    transform.translate(center.x(), center.y());
    transform.rotate(angleDeg);
    return transform.inverted().map(point);
}

} // namespace

RectRoi::RectRoi(const QString& roiId, const QRectF& imageRect)
    : rect(imageRect.normalized())
{
    id = roiId;
    type = RoiType::Rect;
}

QRectF RectRoi::boundingRect() const
{
    return rect;
}

bool RectRoi::hitTest(const QPointF& imagePoint, double imageTolerance) const
{
    return rect.adjusted(-imageTolerance, -imageTolerance, imageTolerance, imageTolerance).contains(imagePoint);
}

void RectRoi::translate(const QPointF& imageDelta)
{
    rect.translate(imageDelta);
}

QVector<QPointF> RectRoi::outlinePoints() const
{
    return {
        rect.topLeft(),
        rect.topRight(),
        rect.bottomRight(),
        rect.bottomLeft(),
        rect.topLeft()
    };
}

QJsonObject RectRoi::toJson() const
{
    QJsonObject object = baseRoiJson(*this, QStringLiteral("Rect"));
    object.insert(QStringLiteral("geometry"), QJsonObject{
        {QStringLiteral("x"), rect.x()},
        {QStringLiteral("y"), rect.y()},
        {QStringLiteral("width"), rect.width()},
        {QStringLiteral("height"), rect.height()}
    });
    return object;
}

RotatedRectRoi::RotatedRectRoi(const QString& roiId,
                               const QPointF& imageCenter,
                               double imageWidth,
                               double imageHeight,
                               double angleDegrees)
    : center(imageCenter)
    , width(imageWidth)
    , height(imageHeight)
    , angleDeg(angleDegrees)
{
    id = roiId;
    type = RoiType::RotatedRect;
}

QRectF RotatedRectRoi::boundingRect() const
{
    return QPolygonF(outlinePoints()).boundingRect();
}

bool RotatedRectRoi::hitTest(const QPointF& imagePoint, double imageTolerance) const
{
    const QPointF localPoint = rotateIntoLocal(imagePoint, center, angleDeg);
    const QRectF localRect(-width * 0.5, -height * 0.5, width, height);
    return localRect.adjusted(-imageTolerance, -imageTolerance, imageTolerance, imageTolerance).contains(localPoint);
}

void RotatedRectRoi::translate(const QPointF& imageDelta)
{
    center += imageDelta;
}

QVector<QPointF> RotatedRectRoi::outlinePoints() const
{
    return rotatedRectPoints(center, width, height, angleDeg);
}

QJsonObject RotatedRectRoi::toJson() const
{
    QJsonObject object = baseRoiJson(*this, QStringLiteral("RotatedRect"));
    object.insert(QStringLiteral("geometry"), QJsonObject{
        {QStringLiteral("center"), pointArray(center)},
        {QStringLiteral("width"), width},
        {QStringLiteral("height"), height},
        {QStringLiteral("angleDeg"), angleDeg}
    });
    return object;
}

CircleRoi::CircleRoi(const QString& roiId, const QPointF& imageCenter, double imageRadius)
    : center(imageCenter)
    , radius(imageRadius)
{
    id = roiId;
    type = RoiType::Circle;
}

QRectF CircleRoi::boundingRect() const
{
    return QRectF(center.x() - radius, center.y() - radius, radius * 2.0, radius * 2.0);
}

bool CircleRoi::hitTest(const QPointF& imagePoint, double imageTolerance) const
{
    return QLineF(center, imagePoint).length() <= radius + imageTolerance;
}

void CircleRoi::translate(const QPointF& imageDelta)
{
    center += imageDelta;
}

QVector<QPointF> CircleRoi::outlinePoints() const
{
    QVector<QPointF> points;
    points.reserve(circleSegments + 1);
    for (int i = 0; i <= circleSegments; ++i) {
        const double angle = (static_cast<double>(i) / circleSegments) * 2.0 * pi;
        points.push_back(QPointF(center.x() + std::cos(angle) * radius,
                                 center.y() + std::sin(angle) * radius));
    }
    return points;
}

QJsonObject CircleRoi::toJson() const
{
    QJsonObject object = baseRoiJson(*this, QStringLiteral("Circle"));
    object.insert(QStringLiteral("geometry"), QJsonObject{
        {QStringLiteral("center"), pointArray(center)},
        {QStringLiteral("radius"), radius}
    });
    return object;
}

LineRoi::LineRoi(const QString& roiId, const QLineF& imageLine)
    : line(imageLine)
{
    id = roiId;
    type = RoiType::Line;
}

QRectF LineRoi::boundingRect() const
{
    return QRectF(line.p1(), line.p2()).normalized();
}

bool LineRoi::hitTest(const QPointF& imagePoint, double imageTolerance) const
{
    return pointLineDistance(imagePoint, line) <= imageTolerance;
}

void LineRoi::translate(const QPointF& imageDelta)
{
    line.translate(imageDelta);
}

QVector<QPointF> LineRoi::outlinePoints() const
{
    return {line.p1(), line.p2()};
}

QJsonObject LineRoi::toJson() const
{
    QJsonObject object = baseRoiJson(*this, QStringLiteral("Line"));
    object.insert(QStringLiteral("geometry"), QJsonObject{
        {QStringLiteral("p1"), pointArray(line.p1())},
        {QStringLiteral("p2"), pointArray(line.p2())}
    });
    return object;
}

} // namespace VisionDisplay
