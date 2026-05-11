#pragma once

#include "VisionDisplay/VisionDisplay_global.h"

#include <QColor>
#include <QJsonObject>
#include <QLineF>
#include <QPointF>
#include <QRectF>
#include <QString>
#include <QVector>

namespace VisionDisplay {

enum class RoiType {
    Rect,
    RotatedRect,
    Circle,
    Line
};

class VISIONDISPLAY_API RoiObject
{
public:
    QString id;
    RoiType type = RoiType::Rect;
    bool visible = true;
    bool selected = false;
    bool locked = false;
    QColor strokeColor = Qt::yellow;
    QColor selectedStrokeColor = QColor(0, 190, 255);

    virtual ~RoiObject() = default;

    virtual QRectF boundingRect() const = 0;
    virtual bool hitTest(const QPointF& imagePoint, double imageTolerance) const = 0;
    virtual void translate(const QPointF& imageDelta) = 0;
    virtual QVector<QPointF> outlinePoints() const = 0;
    virtual QJsonObject toJson() const = 0;
};

class VISIONDISPLAY_API RectRoi final : public RoiObject
{
public:
    explicit RectRoi(const QString& roiId, const QRectF& imageRect);

    QRectF rect;

    QRectF boundingRect() const override;
    bool hitTest(const QPointF& imagePoint, double imageTolerance) const override;
    void translate(const QPointF& imageDelta) override;
    QVector<QPointF> outlinePoints() const override;
    QJsonObject toJson() const override;
};

class VISIONDISPLAY_API RotatedRectRoi final : public RoiObject
{
public:
    RotatedRectRoi(const QString& roiId,
                   const QPointF& imageCenter,
                   double imageWidth,
                   double imageHeight,
                   double angleDegrees);

    QPointF center;
    double width = 0.0;
    double height = 0.0;
    double angleDeg = 0.0;

    QRectF boundingRect() const override;
    bool hitTest(const QPointF& imagePoint, double imageTolerance) const override;
    void translate(const QPointF& imageDelta) override;
    QVector<QPointF> outlinePoints() const override;
    QJsonObject toJson() const override;
};

class VISIONDISPLAY_API CircleRoi final : public RoiObject
{
public:
    CircleRoi(const QString& roiId, const QPointF& imageCenter, double imageRadius);

    QPointF center;
    double radius = 0.0;

    QRectF boundingRect() const override;
    bool hitTest(const QPointF& imagePoint, double imageTolerance) const override;
    void translate(const QPointF& imageDelta) override;
    QVector<QPointF> outlinePoints() const override;
    QJsonObject toJson() const override;
};

class VISIONDISPLAY_API LineRoi final : public RoiObject
{
public:
    LineRoi(const QString& roiId, const QLineF& imageLine);

    QLineF line;

    QRectF boundingRect() const override;
    bool hitTest(const QPointF& imagePoint, double imageTolerance) const override;
    void translate(const QPointF& imageDelta) override;
    QVector<QPointF> outlinePoints() const override;
    QJsonObject toJson() const override;
};

} // namespace VisionDisplay
