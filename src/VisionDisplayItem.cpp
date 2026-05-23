#include "VisionDisplay/VisionDisplayItem.h"

#include <QHoverEvent>
#include <QFile>
#include <QFont>
#include <QFontMetrics>
#include <QImageReader>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QKeyEvent>
#include <QMetaObject>
#include <QMouseEvent>
#include <QIODevice>
#include <QPainter>
#include <QPolygonF>
#include <QSGFlatColorMaterial>
#include <QSGGeometryNode>
#include <QSGSimpleTextureNode>
#include <QSGTexture>
#include <QQuickWindow>
#include <QThread>
#include <QUrl>
#include <QVariantMap>
#include <QWheelEvent>

#include <algorithm>
#include <cstring>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>

namespace VisionDisplay {

namespace {

constexpr int defaultGraphicLayer = static_cast<int>(DisplayLayer::Result);
constexpr int circleSegments = 96;
constexpr double pi = 3.14159265358979323846;
constexpr int maxGeometryPoints = 100000;
constexpr double minModelRoiSize = 32.0;

bool isFiniteValue(double value)
{
    return std::isfinite(value);
}

bool isFinitePoint(const QPointF& point)
{
    return isFiniteValue(point.x()) && isFiniteValue(point.y());
}

bool isFiniteRect(const QRectF& rect)
{
    return isFinitePoint(rect.topLeft())
        && isFiniteValue(rect.width())
        && isFiniteValue(rect.height());
}

class DisplayRootNode final : public QSGNode
{
public:
    DisplayRootNode()
    {
        imageNode = new QSGSimpleTextureNode;
        imageNode->setOwnsTexture(false);
        overlayNode = new QSGNode;
        appendChildNode(imageNode);
        appendChildNode(overlayNode);
    }

    ~DisplayRootNode() override
    {
        if (imageNode) {
            removeChildNode(imageNode);
            delete imageNode;
            imageNode = nullptr;
        }
        delete imageTexture;
        imageTexture = nullptr;
    }

    QSGSimpleTextureNode* imageNode = nullptr;
    QSGNode* overlayNode = nullptr;
    QSGTexture* imageTexture = nullptr;
};

void clearChildNodes(QSGNode* node)
{
    while (QSGNode* child = node->firstChild()) {
        node->removeChildNode(child);
        delete child;
    }
}

int bytesPerPixel(PixelFormat pixelFormat)
{
    switch (pixelFormat) {
    case PixelFormat::Gray8:
        return 1;
    case PixelFormat::RGB888:
    case PixelFormat::BGR888:
        return 3;
    case PixelFormat::RGBA8888:
    case PixelFormat::BGRA8888:
        return 4;
    }
    return 0;
}

QSGGeometryNode* createLineNode(const QVector<QPointF>& viewPoints,
                                unsigned int drawingMode,
                                const GraphicStyle& style,
                                double zoom)
{
    if (viewPoints.size() < 2 || viewPoints.size() > maxGeometryPoints) {
        return nullptr;
    }
    if (drawingMode == QSGGeometry::DrawLines && (viewPoints.size() % 2) != 0) {
        return nullptr;
    }
    for (const QPointF& point : viewPoints) {
        if (!isFinitePoint(point)) {
            return nullptr;
        }
    }

    auto* node = new QSGGeometryNode;
    auto* geometry = new QSGGeometry(QSGGeometry::defaultAttributes_Point2D(), viewPoints.size());
    geometry->setDrawingMode(drawingMode);
    const double lineWidth = style.lineWidthInViewPixels ? style.lineWidth : style.lineWidth * zoom;
    geometry->setLineWidth(static_cast<float>(std::max(1.0, lineWidth)));

    QSGGeometry::Point2D* vertices = geometry->vertexDataAsPoint2D();
    for (qsizetype i = 0; i < viewPoints.size(); ++i) {
        vertices[i].set(static_cast<float>(viewPoints[i].x()), static_cast<float>(viewPoints[i].y()));
    }

    auto* material = new QSGFlatColorMaterial;
    QColor color = style.strokeColor;
    color.setAlphaF(std::clamp(style.opacity, 0.0, 1.0));
    material->setColor(color);

    node->setGeometry(geometry);
    node->setMaterial(material);
    node->setFlag(QSGNode::OwnsGeometry);
    node->setFlag(QSGNode::OwnsMaterial);
    return node;
}

QVector<QPointF> rectPoints(const QRectF& rect)
{
    return {
        rect.topLeft(),
        rect.topRight(),
        rect.bottomRight(),
        rect.bottomLeft(),
        rect.topLeft()
    };
}

QVector<QPointF> circlePoints(const QPointF& center, double radius)
{
    if (!isFinitePoint(center) || !isFiniteValue(radius) || radius <= 0.0) {
        return {};
    }
    QVector<QPointF> points;
    points.reserve(circleSegments + 1);
    for (int i = 0; i <= circleSegments; ++i) {
        const double angle = (static_cast<double>(i) / circleSegments) * 2.0 * pi;
        points.push_back(QPointF(center.x() + std::cos(angle) * radius,
                                 center.y() + std::sin(angle) * radius));
    }
    return points;
}

QVector<QPointF> crossPoints(const QPointF& center, double size)
{
    if (!isFinitePoint(center) || !isFiniteValue(size) || size <= 0.0) {
        return {};
    }
    const double halfSize = size * 0.5;
    return {
        QPointF(center.x() - halfSize, center.y()),
        QPointF(center.x() + halfSize, center.y()),
        QPointF(center.x(), center.y() - halfSize),
        QPointF(center.x(), center.y() + halfSize)
    };
}

QVector<QPointF> rotatedRectPoints(const QPointF& center, double width, double height, double angleDeg)
{
    if (!isFinitePoint(center) || !isFiniteValue(width) || !isFiniteValue(height) || !isFiniteValue(angleDeg)
        || width <= 0.0 || height <= 0.0) {
        return {};
    }
    const double halfWidth = width * 0.5;
    const double halfHeight = height * 0.5;
    const double rad = angleDeg * pi / 180.0;
    const double c = std::cos(rad);
    const double s = std::sin(rad);
    const QVector<QPointF> local{
        QPointF(-halfWidth, -halfHeight),
        QPointF(halfWidth, -halfHeight),
        QPointF(halfWidth, halfHeight),
        QPointF(-halfWidth, halfHeight),
        QPointF(-halfWidth, -halfHeight)
    };

    QVector<QPointF> points;
    points.reserve(local.size());
    for (const QPointF& point : local) {
        points.push_back(QPointF(center.x() + point.x() * c - point.y() * s,
                                 center.y() + point.x() * s + point.y() * c));
    }
    return points;
}

QVector<QPointF> arcPoints(const QPointF& center, double radius, double startAngleDeg, double spanAngleDeg)
{
    if (!isFinitePoint(center) || !isFiniteValue(radius) || !isFiniteValue(startAngleDeg) || !isFiniteValue(spanAngleDeg)
        || radius <= 0.0 || std::abs(spanAngleDeg) <= 0.0001) {
        return {};
    }
    const int segments = std::clamp(static_cast<int>(std::ceil(std::abs(spanAngleDeg) / 6.0)), 12, 720);
    QVector<QPointF> points;
    points.reserve(segments + 1);
    for (int i = 0; i <= segments; ++i) {
        const double angleDeg = startAngleDeg + spanAngleDeg * static_cast<double>(i) / segments;
        const double rad = angleDeg * pi / 180.0;
        points.push_back(QPointF(center.x() + std::cos(rad) * radius,
                                 center.y() + std::sin(rad) * radius));
    }
    return points;
}

QVector<QPointF> ellipsePoints(const QPointF& center,
                               double radiusA,
                               double radiusB,
                               double angleDeg,
                               double startAngleDeg,
                               double spanAngleDeg)
{
    if (!isFinitePoint(center) || !isFiniteValue(radiusA) || !isFiniteValue(radiusB) || !isFiniteValue(angleDeg)
        || !isFiniteValue(startAngleDeg) || !isFiniteValue(spanAngleDeg)
        || radiusA <= 0.0 || radiusB <= 0.0 || std::abs(spanAngleDeg) <= 0.0001) {
        return {};
    }
    const int segments = std::clamp(static_cast<int>(std::ceil(std::abs(spanAngleDeg) / 5.0)), 24, 720);
    const double angleRad = angleDeg * pi / 180.0;
    const double c = std::cos(angleRad);
    const double s = std::sin(angleRad);
    QVector<QPointF> points;
    points.reserve(segments + 1);
    for (int i = 0; i <= segments; ++i) {
        const double tDeg = startAngleDeg + spanAngleDeg * static_cast<double>(i) / segments;
        const double t = tDeg * pi / 180.0;
        const double xLocal = std::cos(t) * radiusA;
        const double yLocal = std::sin(t) * radiusB;
        points.push_back(QPointF(center.x() + c * xLocal - s * yLocal,
                                 center.y() + s * xLocal + c * yLocal));
    }
    return points;
}

QVector<QPointF> mapPoints(const QVector<QPointF>& imagePoints, const CoordinateMapper& mapper)
{
    QVector<QPointF> viewPoints;
    viewPoints.reserve(imagePoints.size());
    for (const QPointF& point : imagePoints) {
        if (!isFinitePoint(point)) {
            continue;
        }
        const QPointF viewPoint = mapper.imageToView(point);
        if (isFinitePoint(viewPoint)) {
            viewPoints.push_back(viewPoint);
        }
    }
    return viewPoints;
}

QSGGeometryNode* createPointMarkerNode(const QPointF& viewPoint,
                                       const GraphicStyle& style,
                                       double markerSize)
{
    const double half = std::max(2.0, markerSize) * 0.5;
    return createLineNode({QPointF(viewPoint.x() - half, viewPoint.y()),
                           QPointF(viewPoint.x() + half, viewPoint.y()),
                           QPointF(viewPoint.x(), viewPoint.y() - half),
                           QPointF(viewPoint.x(), viewPoint.y() + half)},
                          QSGGeometry::DrawLines,
                          style,
                          1.0);
}

QSGGeometryNode* createArrowNode(const QLineF& imageLine,
                                 const CoordinateMapper& mapper,
                                 const GraphicStyle& style,
                                 double arrowHeadSize)
{
    const QPointF p1 = mapper.imageToView(imageLine.p1());
    const QPointF p2 = mapper.imageToView(imageLine.p2());
    if (!isFinitePoint(p1) || !isFinitePoint(p2)) {
        return nullptr;
    }
    QLineF shaft(p1, p2);
    if (shaft.length() <= 0.0001) {
        return nullptr;
    }

    const double head = std::max(4.0, arrowHeadSize);
    const double angle = std::atan2(p2.y() - p1.y(), p2.x() - p1.x());
    const double sideA = angle + pi * 0.82;
    const double sideB = angle - pi * 0.82;
    const QPointF h1(p2.x() + std::cos(sideA) * head, p2.y() + std::sin(sideA) * head);
    const QPointF h2(p2.x() + std::cos(sideB) * head, p2.y() + std::sin(sideB) * head);
    return createLineNode({p1, p2, p2, h1, p2, h2}, QSGGeometry::DrawLines, style, 1.0);
}

QVector<QPointF> closedPoints(QVector<QPointF> points)
{
    if (points.size() > 2 && points.first() != points.last()) {
        points.push_back(points.first());
    }
    return points;
}

QSGSimpleTextureNode* createTextNode(const GraphicObject& graphic,
                                     const CoordinateMapper& mapper,
                                     QQuickWindow* window,
                                     double zoom)
{
    if (graphic.text.isEmpty() || !window) {
        return nullptr;
    }

    const int fontPixelSize = std::max(1, graphic.style.textSizeInViewPixels
                                              ? graphic.style.fontPixelSize
                                              : static_cast<int>(std::round(graphic.style.fontPixelSize * zoom)));
    QFont font;
    font.setPixelSize(fontPixelSize);

    const QFontMetrics metrics(font);
    const QRect textBounds = metrics.boundingRect(graphic.text).adjusted(-2, -2, 2, 2);
    if (textBounds.width() <= 0 || textBounds.height() <= 0) {
        return nullptr;
    }
    QImage textImage(textBounds.size(), QImage::Format_RGBA8888_Premultiplied);
    if (textImage.isNull()) {
        return nullptr;
    }
    textImage.fill(Qt::transparent);

    QPainter painter(&textImage);
    painter.setFont(font);
    QColor color = graphic.style.strokeColor;
    color.setAlphaF(std::clamp(graphic.style.opacity, 0.0, 1.0));
    painter.setPen(color);
    painter.drawText(QPoint(2 - textBounds.left(), 2 - textBounds.top()), graphic.text);
    painter.end();

    auto* node = new QSGSimpleTextureNode;
    node->setOwnsTexture(true);
    QSGTexture* texture = window->createTextureFromImage(textImage);
    if (!texture) {
        delete node;
        return nullptr;
    }
    node->setTexture(texture);

    const QPointF topLeft = mapper.imageToView(graphic.textPosition);
    if (!isFinitePoint(topLeft)) {
        delete node;
        return nullptr;
    }
    node->setRect(QRectF(topLeft, QSizeF(textImage.width(), textImage.height())));
    return node;
}

QSGNode* createGraphicLabelNode(const GraphicObject& graphic,
                                const QPointF& imagePosition,
                                const CoordinateMapper& mapper,
                                QQuickWindow* window)
{
    if (graphic.label.isEmpty()) {
        return nullptr;
    }

    GraphicObject textGraphic;
    textGraphic.type = GraphicType::Text;
    textGraphic.textPosition = imagePosition;
    textGraphic.text = graphic.label;
    textGraphic.style = graphic.style;
    return createTextNode(textGraphic, mapper, window, mapper.zoom());
}

GraphicStyle roiStyle(const RoiObject& roi)
{
    GraphicStyle style;
    style.strokeColor = roi.selected ? roi.selectedStrokeColor : roi.strokeColor;
    style.lineWidth = roi.selected ? 2.5 : 1.5;
    style.lineWidthInViewPixels = true;
    return style;
}

GraphicStyle resultStyle(const QColor& color, double lineWidth = 2.0, int fontPixelSize = 16)
{
    GraphicStyle style;
    style.strokeColor = color;
    style.lineWidth = lineWidth;
    style.fontPixelSize = fontPixelSize;
    style.lineWidthInViewPixels = true;
    style.textSizeInViewPixels = true;
    return style;
}

GraphicStyle toolStyle(const QColor& color, double lineWidth = 1.5, int fontPixelSize = 14)
{
    GraphicStyle style;
    style.strokeColor = color;
    style.lineWidth = lineWidth;
    style.fontPixelSize = fontPixelSize;
    style.lineWidthInViewPixels = true;
    style.textSizeInViewPixels = true;
    return style;
}

GraphicObject baseToolGraphic(const QString& toolId,
                              const QString& id,
                              const QString& toolType,
                              GraphicType graphicType,
                              const QColor& color,
                              bool ok = true)
{
    GraphicObject graphic;
    graphic.id = id;
    graphic.type = graphicType;
    graphic.layer = static_cast<int>(DisplayLayer::Measure);
    graphic.resultGraphic = false;
    graphic.toolGraphic = true;
    graphic.toolId = toolId;
    graphic.toolName = toolId;
    graphic.toolType = toolType;
    graphic.statusOk = ok;
    graphic.style = toolStyle(ok ? color : QColor(255, 80, 80), 1.7, 14);
    return graphic;
}

QPointF directionPoint(const QPointF& center, double angleDeg, double imageLength)
{
    const double rad = angleDeg * pi / 180.0;
    return QPointF(center.x() + std::cos(rad) * imageLength,
                   center.y() + std::sin(rad) * imageLength);
}

QPointF rotatedVector(double x, double y, double angleDeg)
{
    const double rad = angleDeg * pi / 180.0;
    const double c = std::cos(rad);
    const double s = std::sin(rad);
    return QPointF(x * c - y * s, x * s + y * c);
}

double vectorAngleDeg(const QPointF& vector)
{
    return std::atan2(vector.y(), vector.x()) * 180.0 / pi;
}

double pointDistance(const QPointF& a, const QPointF& b)
{
    return QLineF(a, b).length();
}

QPointF unrotateVector(const QPointF& vector, double angleDeg)
{
    const double rad = -angleDeg * pi / 180.0;
    const double c = std::cos(rad);
    const double s = std::sin(rad);
    return QPointF(vector.x() * c - vector.y() * s,
                   vector.x() * s + vector.y() * c);
}

bool pointInRotatedRect(const QPointF& point,
                        const QPointF& center,
                        double width,
                        double height,
                        double angleDeg,
                        double tolerance)
{
    const QPointF local = unrotateVector(point - center, angleDeg);
    return std::abs(local.x()) <= width * 0.5 + tolerance
        && std::abs(local.y()) <= height * 0.5 + tolerance;
}

QVector<QPointF> rectHandlePoints(const QRectF& rect)
{
    const QRectF r = rect.normalized();
    const QPointF center = r.center();
    return {
        r.topLeft(),
        QPointF(center.x(), r.top()),
        r.topRight(),
        QPointF(r.right(), center.y()),
        r.bottomRight(),
        QPointF(center.x(), r.bottom()),
        r.bottomLeft(),
        QPointF(r.left(), center.y())
    };
}

QVector<QPointF> edgeHandlePoints(const QRectF& rect)
{
    const QRectF r = rect.normalized();
    const QPointF center = r.center();
    return {
        QPointF(center.x(), r.top()),
        QPointF(r.right(), center.y()),
        QPointF(center.x(), r.bottom()),
        QPointF(r.left(), center.y())
    };
}

int rectHandleAt(const QRectF& rect, const QPointF& point, double tolerance)
{
    const QVector<QPointF> handles = rectHandlePoints(rect);
    for (int i = 0; i < handles.size(); ++i) {
        if (QLineF(point, handles[i]).length() <= tolerance) {
            return i;
        }
    }
    return -1;
}

QRectF resizedRectFromHandle(const QRectF& original, int handle, const QPointF& currentPoint)
{
    QRectF rect = original.normalized();
    double left = rect.left();
    double right = rect.right();
    double top = rect.top();
    double bottom = rect.bottom();

    switch (handle) {
    case 0:
        left = currentPoint.x();
        top = currentPoint.y();
        break;
    case 1:
        top = currentPoint.y();
        break;
    case 2:
        right = currentPoint.x();
        top = currentPoint.y();
        break;
    case 3:
        right = currentPoint.x();
        break;
    case 4:
        right = currentPoint.x();
        bottom = currentPoint.y();
        break;
    case 5:
        bottom = currentPoint.y();
        break;
    case 6:
        left = currentPoint.x();
        bottom = currentPoint.y();
        break;
    case 7:
        left = currentPoint.x();
        break;
    default:
        return original.translated(currentPoint - original.center());
    }

    constexpr double minSize = 3.0;
    QRectF resized(QPointF(left, top), QPointF(right, bottom));
    resized = resized.normalized();
    if (resized.width() < minSize) {
        resized.setWidth(minSize);
    }
    if (resized.height() < minSize) {
        resized.setHeight(minSize);
    }
    return resized;
}

QString metricText(const QString& label, double value, const QString& unit)
{
    return QStringLiteral("%1 = %2 %3").arg(label).arg(value, 0, 'f', 3).arg(unit);
}

QString graphicTypeName(GraphicType type)
{
    switch (type) {
    case GraphicType::Line: return QStringLiteral("Line");
    case GraphicType::Rect: return QStringLiteral("Rect");
    case GraphicType::Circle: return QStringLiteral("Circle");
    case GraphicType::Text: return QStringLiteral("Text");
    case GraphicType::Cross: return QStringLiteral("Cross");
    case GraphicType::Polyline: return QStringLiteral("Polyline");
    case GraphicType::DefectBox: return QStringLiteral("DefectBox");
    case GraphicType::DefectContour: return QStringLiteral("DefectContour");
    case GraphicType::BlobRegion: return QStringLiteral("BlobRegion");
    case GraphicType::MatchContour: return QStringLiteral("MatchContour");
    case GraphicType::FittedLine: return QStringLiteral("FittedLine");
    case GraphicType::FittedCircle: return QStringLiteral("FittedCircle");
    case GraphicType::StatusText: return QStringLiteral("StatusText");
    case GraphicType::RotatedRect: return QStringLiteral("RotatedRect");
    case GraphicType::Arc: return QStringLiteral("Arc");
    case GraphicType::Arrow: return QStringLiteral("Arrow");
    case GraphicType::PointMarker: return QStringLiteral("PointMarker");
    }
    return QStringLiteral("Line");
}

QJsonArray jsonPoint(const QPointF& point)
{
    QJsonArray array;
    array.append(point.x());
    array.append(point.y());
    return array;
}

QJsonArray jsonPoints(const QVector<QPointF>& points)
{
    QJsonArray array;
    for (const QPointF& point : points) {
        array.append(jsonPoint(point));
    }
    return array;
}

QPointF pointFromJson(const QJsonValue& value)
{
    const QJsonArray array = value.toArray();
    if (array.size() >= 2) {
        return QPointF(array.at(0).toDouble(), array.at(1).toDouble());
    }
    const QJsonObject object = value.toObject();
    return QPointF(object.value(QStringLiteral("x")).toDouble(),
                   object.value(QStringLiteral("y")).toDouble());
}

QJsonObject styleToJson(const GraphicStyle& style)
{
    return {
        {QStringLiteral("strokeColor"), style.strokeColor.name(QColor::HexArgb)},
        {QStringLiteral("fillColor"), style.fillColor.name(QColor::HexArgb)},
        {QStringLiteral("lineWidth"), style.lineWidth},
        {QStringLiteral("opacity"), style.opacity},
        {QStringLiteral("fontPixelSize"), style.fontPixelSize},
        {QStringLiteral("lineWidthInViewPixels"), style.lineWidthInViewPixels},
        {QStringLiteral("textSizeInViewPixels"), style.textSizeInViewPixels}
    };
}

QJsonObject graphicGeometryToJson(const GraphicObject& graphic)
{
    switch (graphic.type) {
    case GraphicType::Line:
    case GraphicType::FittedLine:
        return {{QStringLiteral("p1"), jsonPoint(graphic.line.p1())},
                {QStringLiteral("p2"), jsonPoint(graphic.line.p2())}};
    case GraphicType::Rect:
    case GraphicType::DefectBox:
        return {{QStringLiteral("x"), graphic.rect.x()},
                {QStringLiteral("y"), graphic.rect.y()},
                {QStringLiteral("width"), graphic.rect.width()},
                {QStringLiteral("height"), graphic.rect.height()}};
    case GraphicType::RotatedRect:
        return {{QStringLiteral("center"), jsonPoint(graphic.center)},
                {QStringLiteral("width"), graphic.rect.width()},
                {QStringLiteral("height"), graphic.rect.height()},
                {QStringLiteral("angleDeg"), graphic.angleDeg}};
    case GraphicType::Circle:
    case GraphicType::FittedCircle:
        return {{QStringLiteral("center"), jsonPoint(graphic.center)},
                {QStringLiteral("radius"), graphic.radius}};
    case GraphicType::Arc:
        return {{QStringLiteral("center"), jsonPoint(graphic.center)},
                {QStringLiteral("radius"), graphic.radius},
                {QStringLiteral("startAngleDeg"), graphic.startAngleDeg},
                {QStringLiteral("spanAngleDeg"), graphic.spanAngleDeg}};
    case GraphicType::Text:
    case GraphicType::StatusText:
        return {{QStringLiteral("position"), jsonPoint(graphic.textPosition)},
                {QStringLiteral("text"), graphic.text}};
    case GraphicType::Cross:
        return {{QStringLiteral("center"), jsonPoint(graphic.crossCenter)},
                {QStringLiteral("size"), graphic.crossSize}};
    case GraphicType::Arrow:
        return {{QStringLiteral("p1"), jsonPoint(graphic.line.p1())},
                {QStringLiteral("p2"), jsonPoint(graphic.line.p2())}};
    case GraphicType::PointMarker:
        return {{QStringLiteral("center"), jsonPoint(graphic.center)},
                {QStringLiteral("markerSize"), graphic.markerSize}};
    case GraphicType::Polyline:
    case GraphicType::DefectContour:
    case GraphicType::BlobRegion:
    case GraphicType::MatchContour:
        return {{QStringLiteral("points"), jsonPoints(graphic.points)}};
    }
    return {};
}

QJsonObject graphicToJson(const GraphicObject& graphic)
{
    QJsonObject object;
    object.insert(QStringLiteral("id"), graphic.id);
    object.insert(QStringLiteral("type"), graphicTypeName(graphic.type));
    object.insert(QStringLiteral("layer"), graphic.layer);
    object.insert(QStringLiteral("visible"), graphic.visible);
    object.insert(QStringLiteral("resultGraphic"), graphic.resultGraphic);
    object.insert(QStringLiteral("toolGraphic"), graphic.toolGraphic);
    if (!graphic.toolId.isEmpty()) {
        object.insert(QStringLiteral("toolId"), graphic.toolId);
    }
    if (!graphic.toolType.isEmpty()) {
        object.insert(QStringLiteral("toolType"), graphic.toolType);
    }
    object.insert(QStringLiteral("geometry"), graphicGeometryToJson(graphic));
    object.insert(QStringLiteral("style"), styleToJson(graphic.style));
    if (!graphic.label.isEmpty()) {
        object.insert(QStringLiteral("label"), graphic.label);
    }
    if (graphic.type == GraphicType::StatusText) {
        object.insert(QStringLiteral("statusOk"), graphic.statusOk);
    }
    return object;
}

bool writeJsonFile(const QString& path, const QJsonDocument& document)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        return false;
    }
    return file.write(document.toJson(QJsonDocument::Indented)) >= 0;
}

bool readJsonFile(const QString& path, QJsonDocument* document)
{
    QFile file(path);
    if (!document || !file.open(QIODevice::ReadOnly)) {
        return false;
    }
    QJsonParseError error;
    *document = QJsonDocument::fromJson(file.readAll(), &error);
    return error.error == QJsonParseError::NoError && !document->isNull();
}

QPen painterPen(const GraphicStyle& style, double zoom)
{
    QColor color = style.strokeColor;
    color.setAlphaF(std::clamp(style.opacity, 0.0, 1.0));
    const double lineWidth = style.lineWidthInViewPixels ? style.lineWidth : style.lineWidth * zoom;
    return QPen(color, std::max(1.0, lineWidth));
}

void drawTextGraphic(QPainter* painter, const GraphicObject& graphic, const CoordinateMapper& mapper)
{
    QFont font = painter->font();
    font.setPixelSize(std::max(1, graphic.style.textSizeInViewPixels
                                      ? graphic.style.fontPixelSize
                                      : static_cast<int>(std::round(graphic.style.fontPixelSize * mapper.zoom()))));
    painter->setFont(font);
    painter->setPen(painterPen(graphic.style, mapper.zoom()).color());
    painter->drawText(mapper.imageToView(graphic.textPosition), graphic.text);
}

void drawPolyline(QPainter* painter,
                  const QVector<QPointF>& imagePoints,
                  const CoordinateMapper& mapper,
                  bool closed)
{
    QVector<QPointF> viewPoints = mapPoints(imagePoints, mapper);
    if (closed) {
        viewPoints = closedPoints(viewPoints);
    }
    if (viewPoints.size() >= 2) {
        painter->drawPolyline(QPolygonF(viewPoints));
    }
}

std::unique_ptr<RoiObject> roiFromJson(const QJsonObject& object)
{
    const QString id = object.value(QStringLiteral("id")).toString();
    const QString type = object.value(QStringLiteral("type")).toString();
    const QJsonObject geometry = object.contains(QStringLiteral("geometry"))
        ? object.value(QStringLiteral("geometry")).toObject()
        : object;

    std::unique_ptr<RoiObject> roi;
    if (type == QStringLiteral("Rect")) {
        roi = std::make_unique<RectRoi>(id,
                                        QRectF(geometry.value(QStringLiteral("x")).toDouble(),
                                               geometry.value(QStringLiteral("y")).toDouble(),
                                               geometry.value(QStringLiteral("width")).toDouble(),
                                               geometry.value(QStringLiteral("height")).toDouble()));
    } else if (type == QStringLiteral("RotatedRect")) {
        roi = std::make_unique<RotatedRectRoi>(id,
                                              pointFromJson(geometry.value(QStringLiteral("center"))),
                                              geometry.value(QStringLiteral("width")).toDouble(),
                                              geometry.value(QStringLiteral("height")).toDouble(),
                                              geometry.value(QStringLiteral("angleDeg")).toDouble());
    } else if (type == QStringLiteral("Circle")) {
        roi = std::make_unique<CircleRoi>(id,
                                         pointFromJson(geometry.value(QStringLiteral("center"))),
                                         geometry.value(QStringLiteral("radius")).toDouble());
    } else if (type == QStringLiteral("Line")) {
        roi = std::make_unique<LineRoi>(id,
                                       QLineF(pointFromJson(geometry.value(QStringLiteral("p1"))),
                                              pointFromJson(geometry.value(QStringLiteral("p2")))));
    }

    if (!roi || roi->id.isEmpty()) {
        return nullptr;
    }

    roi->visible = object.value(QStringLiteral("visible")).toBool(true);
    roi->locked = object.value(QStringLiteral("locked")).toBool(false);
    const QJsonObject style = object.value(QStringLiteral("style")).toObject();
    if (style.contains(QStringLiteral("strokeColor"))) {
        roi->strokeColor = QColor(style.value(QStringLiteral("strokeColor")).toString());
    }
    if (style.contains(QStringLiteral("selectedStrokeColor"))) {
        roi->selectedStrokeColor = QColor(style.value(QStringLiteral("selectedStrokeColor")).toString());
    }
    return roi;
}

} // namespace

VisionDisplayItem::VisionDisplayItem(QQuickItem* parent)
    : QQuickItem(parent)
{
    setFlag(ItemHasContents, true);
    setFlag(ItemIsFocusScope, true);
    setClip(true);
    setAcceptedMouseButtons(Qt::LeftButton | Qt::MiddleButton | Qt::RightButton);
    setAcceptHoverEvents(true);
}

VisionDisplayItem::~VisionDisplayItem() = default;

double VisionDisplayItem::zoom() const
{
    return m_mapper.zoom();
}

void VisionDisplayItem::setZoom(double zoom)
{
    const double nextZoom = boundedZoom(zoom);
    if (qFuzzyCompare(m_mapper.zoom(), nextZoom)) {
        return;
    }

    m_mapper.setZoom(nextZoom);
    emit zoomChanged();
    markOverlayDirty();
    update();
}

double VisionDisplayItem::minZoom() const
{
    return m_minZoom;
}

void VisionDisplayItem::setMinZoom(double minZoom)
{
    const double nextMinZoom = std::max(minZoom, 0.000001);
    if (qFuzzyCompare(m_minZoom, nextMinZoom)) {
        return;
    }

    m_minZoom = nextMinZoom;
    if (m_maxZoom < m_minZoom) {
        m_maxZoom = m_minZoom;
        emit maxZoomChanged();
    }
    setZoom(m_mapper.zoom());
    emit minZoomChanged();
}

double VisionDisplayItem::maxZoom() const
{
    return m_maxZoom;
}

void VisionDisplayItem::setMaxZoom(double maxZoom)
{
    const double nextMaxZoom = std::max(maxZoom, m_minZoom);
    if (qFuzzyCompare(m_maxZoom, nextMaxZoom)) {
        return;
    }

    m_maxZoom = nextMaxZoom;
    setZoom(m_mapper.zoom());
    emit maxZoomChanged();
}

bool VisionDisplayItem::showPixelInfo() const
{
    return m_showPixelInfo;
}

void VisionDisplayItem::setShowPixelInfo(bool showPixelInfo)
{
    if (m_showPixelInfo == showPixelInfo) {
        return;
    }

    m_showPixelInfo = showPixelInfo;
    emit showPixelInfoChanged();
}

bool VisionDisplayItem::showCrosshair() const
{
    return m_showCrosshair;
}

void VisionDisplayItem::setShowCrosshair(bool showCrosshair)
{
    if (m_showCrosshair == showCrosshair) {
        return;
    }

    m_showCrosshair = showCrosshair;
    emit showCrosshairChanged();
}

int VisionDisplayItem::interactionMode() const
{
    return m_interactionMode;
}

void VisionDisplayItem::setInteractionMode(int interactionMode)
{
    if (m_interactionMode == interactionMode) {
        return;
    }

    m_interactionMode = interactionMode;
    emit interactionModeChanged();
}

QString VisionDisplayItem::selectedRoiId() const
{
    return m_rois.selectedRoiId();
}

bool VisionDisplayItem::autoFitOnNewImage() const
{
    return m_autoFitOnNewImage;
}

void VisionDisplayItem::setAutoFitOnNewImage(bool enabled)
{
    if (m_autoFitOnNewImage == enabled) {
        return;
    }

    m_autoFitOnNewImage = enabled;
    emit autoFitOnNewImageChanged();
}

bool VisionDisplayItem::keepViewTransformOnNewImage() const
{
    return m_keepViewTransformOnNewImage;
}

void VisionDisplayItem::setKeepViewTransformOnNewImage(bool enabled)
{
    if (m_keepViewTransformOnNewImage == enabled) {
        return;
    }

    m_keepViewTransformOnNewImage = enabled;
    emit keepViewTransformOnNewImageChanged();
}

QRectF VisionDisplayItem::modelRoi() const
{
    return m_modelRoi;
}

void VisionDisplayItem::setModelRoi(const QRectF& roi)
{
    const QRectF next = boundedModelRoi(roi);
    if (m_modelRoi == next) {
        return;
    }
    m_modelRoi = next;
    emit modelRoiChanged();
    markOverlayDirty();
    update();
}

bool VisionDisplayItem::modelRoiVisible() const
{
    return m_modelRoiVisible;
}

void VisionDisplayItem::setModelRoiVisible(bool visible)
{
    if (m_modelRoiVisible == visible) {
        return;
    }
    m_modelRoiVisible = visible;
    emit modelRoiVisibleChanged();
    markOverlayDirty();
    update();
}

bool VisionDisplayItem::modelRoiEditable() const
{
    return m_modelRoiEditable;
}

void VisionDisplayItem::setModelRoiEditable(bool editable)
{
    if (m_modelRoiEditable == editable) {
        return;
    }
    m_modelRoiEditable = editable;
    if (!editable) {
        m_modelRoiDrag = ModelRoiDrag();
        unsetCursor();
    }
    emit modelRoiEditableChanged();
    markOverlayDirty();
    update();
}

void VisionDisplayItem::resetModelRoi()
{
    setModelRoi(defaultModelRoi());
    setModelRoiVisible(true);
}

void VisionDisplayItem::setImage(const QImage& image)
{
    updateFrame(image);
}

bool VisionDisplayItem::loadImage(const QString& pathOrUrl)
{
    const QUrl url(pathOrUrl);
    const QString path = url.isLocalFile() ? url.toLocalFile() : pathOrUrl;

    QImageReader reader(path);
    reader.setAutoTransform(true);
    const QImage image = reader.read();
    if (image.isNull()) {
        return false;
    }

    updateFrame(image);
    return true;
}

void VisionDisplayItem::updateFrame(const QImage& image)
{
    FrameData frame = frameFromImage(image);
    if (!frame.isValid()) {
        return;
    }

    if (thread() != QThread::currentThread()) {
        QMetaObject::invokeMethod(this,
                                  [this, frame = std::move(frame)]() mutable {
                                      applyFrame(std::move(frame));
                                  },
                                  Qt::QueuedConnection);
        return;
    }

    applyFrame(std::move(frame));
}

void VisionDisplayItem::updateFrame(const uchar* data, int width, int height, int stride, int pixelFormat)
{
    FrameData frame = frameFromRaw(data, width, height, stride, static_cast<PixelFormat>(pixelFormat));
    if (!frame.isValid()) {
        return;
    }

    if (thread() != QThread::currentThread()) {
        QMetaObject::invokeMethod(this,
                                  [this, frame = std::move(frame)]() mutable {
                                      applyFrame(std::move(frame));
                                  },
                                  Qt::QueuedConnection);
        return;
    }

    applyFrame(std::move(frame));
}

void VisionDisplayItem::updateFrame(const QByteArray& data, int width, int height, int stride, int pixelFormat)
{
    updateFrame(reinterpret_cast<const uchar*>(data.constData()), width, height, stride, pixelFormat);
}

void VisionDisplayItem::fitToWindow()
{
    updateMapperViewSize();
    const double previousZoom = m_mapper.zoom();
    m_mapper.fitToWindow();
    m_mapper.setZoom(boundedZoom(m_mapper.zoom()));

    if (!qFuzzyCompare(previousZoom, m_mapper.zoom())) {
        emit zoomChanged();
    }
    markOverlayDirty();
    update();
}

void VisionDisplayItem::setZoomAt(double zoom, double viewX, double viewY)
{
    const QPointF anchorViewPoint(viewX, viewY);
    const QPointF anchorImagePoint = m_mapper.viewToImage(anchorViewPoint);
    const double nextZoom = boundedZoom(zoom);

    if (qFuzzyCompare(m_mapper.zoom(), nextZoom)) {
        return;
    }

    m_mapper.setZoom(nextZoom);
    const QPointF viewAfterZoom = m_mapper.imageToView(anchorImagePoint);
    const QPointF nextOffset = m_mapper.offset() + (anchorViewPoint - viewAfterZoom);
    m_mapper.setOffset(nextOffset.x(), nextOffset.y());

    emit zoomChanged();
    markOverlayDirty();
    update();
}

void VisionDisplayItem::zoomIn()
{
    setZoomAt(m_mapper.zoom() * 1.25, width() * 0.5, height() * 0.5);
}

void VisionDisplayItem::zoomOut()
{
    setZoomAt(m_mapper.zoom() / 1.25, width() * 0.5, height() * 0.5);
}

QPointF VisionDisplayItem::imageToView(const QPointF& imagePoint) const
{
    return m_mapper.imageToView(imagePoint);
}

QPointF VisionDisplayItem::viewToImage(const QPointF& viewPoint) const
{
    return m_mapper.viewToImage(viewPoint);
}

void VisionDisplayItem::clearGraphics()
{
    m_graphics.clear();
    markOverlayDirty();
    update();
}

void VisionDisplayItem::clearGraphicsByLayer(int layer)
{
    m_graphics.clearLayer(layer);
    markOverlayDirty();
    update();
}

void VisionDisplayItem::clearResultGraphics()
{
    m_graphics.clearResultGraphics();
    markOverlayDirty();
    update();
}

void VisionDisplayItem::setLayerVisible(int layer, bool visible)
{
    m_graphics.setLayerVisible(layer, visible);
    markOverlayDirty();
    update();
}

void VisionDisplayItem::clearRois()
{
    const bool hadSelection = !m_rois.selectedRoiId().isEmpty();
    m_draggingRoi = false;
    m_roiDragSnapshot.valid = false;
    m_rois.clear();
    if (hadSelection) {
        emit selectedRoiIdChanged();
        emit roiSelected(QString());
    }
    markOverlayDirty();
    update();
}

void VisionDisplayItem::createRectRoi(const QString& id, double x, double y, double w, double h)
{
    m_rois.addRoi(std::make_unique<RectRoi>(id, QRectF(x, y, w, h)));
    emit selectedRoiIdChanged();
    emit roiCreated(id);
    emit roiSelected(id);
    markOverlayDirty();
    update();
}

void VisionDisplayItem::createRotatedRectRoi(const QString& id, double cx, double cy, double w, double h, double angleDeg)
{
    m_rois.addRoi(std::make_unique<RotatedRectRoi>(id, QPointF(cx, cy), w, h, angleDeg));
    emit selectedRoiIdChanged();
    emit roiCreated(id);
    emit roiSelected(id);
    markOverlayDirty();
    update();
}

void VisionDisplayItem::createCircleRoi(const QString& id, double cx, double cy, double r)
{
    m_rois.addRoi(std::make_unique<CircleRoi>(id, QPointF(cx, cy), r));
    emit selectedRoiIdChanged();
    emit roiCreated(id);
    emit roiSelected(id);
    markOverlayDirty();
    update();
}

void VisionDisplayItem::createLineRoi(const QString& id, double x1, double y1, double x2, double y2)
{
    m_rois.addRoi(std::make_unique<LineRoi>(id, QLineF(QPointF(x1, y1), QPointF(x2, y2))));
    emit selectedRoiIdChanged();
    emit roiCreated(id);
    emit roiSelected(id);
    markOverlayDirty();
    update();
}

void VisionDisplayItem::deleteSelectedRoi()
{
    const QString deletedId = m_rois.selectedRoiId();
    if (deletedId.isEmpty() || !m_rois.removeSelectedRoi()) {
        return;
    }

    m_draggingRoi = false;
    m_roiDragSnapshot.valid = false;
    emit selectedRoiIdChanged();
    emit roiDeleted(deletedId);
    emit roiSelected(QString());
    markOverlayDirty();
    update();
}

QVariantMap VisionDisplayItem::roiGeometry(const QString& id) const
{
    QVariantMap geometry;
    const RoiObject* roi = m_rois.findById(id);
    if (!roi) {
        geometry.insert(QStringLiteral("valid"), false);
        return geometry;
    }

    geometry.insert(QStringLiteral("valid"), true);
    geometry.insert(QStringLiteral("id"), roi->id);
    switch (roi->type) {
    case RoiType::Rect: {
        const QRectF rect = static_cast<const RectRoi*>(roi)->rect.normalized();
        geometry.insert(QStringLiteral("type"), QStringLiteral("rect"));
        geometry.insert(QStringLiteral("x"), rect.x());
        geometry.insert(QStringLiteral("y"), rect.y());
        geometry.insert(QStringLiteral("width"), rect.width());
        geometry.insert(QStringLiteral("height"), rect.height());
        break;
    }
    case RoiType::RotatedRect: {
        const auto* rotated = static_cast<const RotatedRectRoi*>(roi);
        geometry.insert(QStringLiteral("type"), QStringLiteral("rotatedRect"));
        geometry.insert(QStringLiteral("centerX"), rotated->center.x());
        geometry.insert(QStringLiteral("centerY"), rotated->center.y());
        geometry.insert(QStringLiteral("width"), rotated->width);
        geometry.insert(QStringLiteral("height"), rotated->height);
        geometry.insert(QStringLiteral("angleDeg"), rotated->angleDeg);
        break;
    }
    case RoiType::Circle: {
        const auto* circle = static_cast<const CircleRoi*>(roi);
        geometry.insert(QStringLiteral("type"), QStringLiteral("circle"));
        geometry.insert(QStringLiteral("centerX"), circle->center.x());
        geometry.insert(QStringLiteral("centerY"), circle->center.y());
        geometry.insert(QStringLiteral("radius"), circle->radius);
        break;
    }
    case RoiType::Line: {
        const QLineF line = static_cast<const LineRoi*>(roi)->line;
        geometry.insert(QStringLiteral("type"), QStringLiteral("line"));
        geometry.insert(QStringLiteral("x1"), line.x1());
        geometry.insert(QStringLiteral("y1"), line.y1());
        geometry.insert(QStringLiteral("x2"), line.x2());
        geometry.insert(QStringLiteral("y2"), line.y2());
        break;
    }
    }
    return geometry;
}

QString VisionDisplayItem::exportRoisJson() const
{
    return exportRoisJsonString();
}

bool VisionDisplayItem::saveImage(const QString& path) const
{
    return !m_image.isNull() && m_image.save(path);
}

bool VisionDisplayItem::saveScreenshot(const QString& path, bool withOverlay) const
{
    if (m_image.isNull() || width() <= 0.0 || height() <= 0.0) {
        return false;
    }

    QImage screenshot(QSize(std::max(1, static_cast<int>(std::ceil(width()))),
                            std::max(1, static_cast<int>(std::ceil(height())))),
                      QImage::Format_RGBA8888_Premultiplied);
    screenshot.fill(Qt::transparent);

    QPainter painter(&screenshot);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.drawImage(imageViewRect(), m_image, QRectF(m_image.rect()));

    if (withOverlay) {
        for (const std::unique_ptr<RoiObject>& roi : m_rois.rois()) {
            if (!roi || !roi->visible) {
                continue;
            }
            painter.setPen(painterPen(roiStyle(*roi), m_mapper.zoom()));
            drawPolyline(&painter, roi->outlinePoints(), m_mapper, false);
            if (roi->selected && roi->type == RoiType::Rect) {
                const QRectF rect = static_cast<const RectRoi*>(roi.get())->rect;
                for (const QPointF& handle : rectHandlePoints(rect)) {
                    const QPointF p = m_mapper.imageToView(handle);
                    constexpr double half = 5.5;
                    painter.drawLine(QPointF(p.x() - half, p.y()), QPointF(p.x() + half, p.y()));
                    painter.drawLine(QPointF(p.x(), p.y() - half), QPointF(p.x(), p.y() + half));
                }
            }
        }

        for (const GraphicObject& graphic : m_graphics.graphics()) {
            if (!graphic.visible || !m_graphics.isLayerVisible(graphic.layer)) {
                continue;
            }
            if (graphic.toolGraphic && !m_graphics.isToolGraphicsVisible(graphic.toolId)) {
                continue;
            }

            painter.setPen(painterPen(graphic.style, m_mapper.zoom()));
            switch (graphic.type) {
            case GraphicType::Line:
            case GraphicType::FittedLine:
                painter.drawLine(m_mapper.imageToView(graphic.line.p1()), m_mapper.imageToView(graphic.line.p2()));
                break;
            case GraphicType::Rect:
            case GraphicType::DefectBox:
                drawPolyline(&painter, rectPoints(graphic.rect), m_mapper, false);
                if (!graphic.label.isEmpty()) {
                    GraphicObject label = graphic;
                    label.type = GraphicType::Text;
                    label.text = graphic.label;
                    label.textPosition = graphic.rect.topLeft() + QPointF(0.0, -18.0 / std::max(0.000001, m_mapper.zoom()));
                    drawTextGraphic(&painter, label, m_mapper);
                }
                break;
            case GraphicType::Circle:
            case GraphicType::FittedCircle:
                drawPolyline(&painter, circlePoints(graphic.center, graphic.radius), m_mapper, false);
                break;
            case GraphicType::Text:
            case GraphicType::StatusText:
                drawTextGraphic(&painter, graphic, m_mapper);
                break;
            case GraphicType::Cross:
                drawPolyline(&painter, crossPoints(graphic.crossCenter, graphic.crossSize), m_mapper, false);
                break;
            case GraphicType::Polyline:
            case GraphicType::MatchContour:
                drawPolyline(&painter, graphic.points, m_mapper, false);
                break;
            case GraphicType::DefectContour:
                drawPolyline(&painter, graphic.points, m_mapper, true);
                if (!graphic.label.isEmpty() && !graphic.points.isEmpty()) {
                    GraphicObject label = graphic;
                    label.type = GraphicType::Text;
                    label.text = graphic.label;
                    label.textPosition = graphic.points.first();
                    drawTextGraphic(&painter, label, m_mapper);
                }
                break;
            case GraphicType::BlobRegion:
                drawPolyline(&painter, graphic.points, m_mapper, true);
                break;
            case GraphicType::RotatedRect:
                drawPolyline(&painter,
                             rotatedRectPoints(graphic.center, graphic.rect.width(), graphic.rect.height(), graphic.angleDeg),
                             m_mapper,
                             false);
                break;
            case GraphicType::Arc:
                drawPolyline(&painter,
                             arcPoints(graphic.center, graphic.radius, graphic.startAngleDeg, graphic.spanAngleDeg),
                             m_mapper,
                             false);
                break;
            case GraphicType::Arrow:
                painter.drawLine(m_mapper.imageToView(graphic.line.p1()), m_mapper.imageToView(graphic.line.p2()));
                break;
            case GraphicType::PointMarker: {
                const QPointF p = m_mapper.imageToView(graphic.center);
                const double half = graphic.markerSize * 0.5;
                painter.drawLine(QPointF(p.x() - half, p.y()), QPointF(p.x() + half, p.y()));
                painter.drawLine(QPointF(p.x(), p.y() - half), QPointF(p.x(), p.y() + half));
                break;
            }
            }
        }
    }

    painter.end();
    return screenshot.save(path);
}

bool VisionDisplayItem::exportGraphicsJson(const QString& path) const
{
    QJsonArray graphics;
    for (const GraphicObject& graphic : m_graphics.graphics()) {
        graphics.append(graphicToJson(graphic));
    }

    const QJsonObject root{
        {QStringLiteral("schema"), QStringLiteral("VisionDisplay.Graphics")},
        {QStringLiteral("version"), 1},
        {QStringLiteral("coordinateSystem"), QStringLiteral("image")},
        {QStringLiteral("graphics"), graphics}
    };
    return writeJsonFile(path, QJsonDocument(root));
}

bool VisionDisplayItem::exportRoisJson(const QString& path) const
{
    return writeJsonFile(path, QJsonDocument::fromJson(exportRoisJsonString().toUtf8()));
}

bool VisionDisplayItem::importRoisJson(const QString& path)
{
    QJsonDocument document;
    if (!readJsonFile(path, &document)) {
        return false;
    }
    return importRoisJsonString(QString::fromUtf8(document.toJson(QJsonDocument::Compact)));
}

QString VisionDisplayItem::exportRoisJsonString() const
{
    const QJsonObject root{
        {QStringLiteral("schema"), QStringLiteral("VisionDisplay.Rois")},
        {QStringLiteral("version"), 1},
        {QStringLiteral("coordinateSystem"), QStringLiteral("image")},
        {QStringLiteral("selectedRoiId"), m_rois.selectedRoiId()},
        {QStringLiteral("rois"), m_rois.toJson()}
    };
    return QString::fromUtf8(QJsonDocument(root).toJson(QJsonDocument::Compact));
}

bool VisionDisplayItem::importRoisJsonString(const QString& json)
{
    QJsonParseError error;
    const QJsonDocument document = QJsonDocument::fromJson(json.toUtf8(), &error);
    if (error.error != QJsonParseError::NoError || document.isNull()) {
        return false;
    }

    const QJsonArray rois = document.isArray()
        ? document.array()
        : document.object().value(QStringLiteral("rois")).toArray();
    const QString previousSelection = m_rois.selectedRoiId();
    m_draggingRoi = false;
    m_roiDragSnapshot.valid = false;
    m_rois.clear();

    for (const QJsonValue& value : rois) {
        if (value.isObject()) {
            m_rois.addRoi(roiFromJson(value.toObject()));
        }
    }

    const QString selectedId = document.object().value(QStringLiteral("selectedRoiId")).toString();
    if (!selectedId.isEmpty()) {
        m_rois.selectRoi(selectedId);
    } else {
        m_rois.clearSelection();
    }

    if (previousSelection != m_rois.selectedRoiId()) {
        emit selectedRoiIdChanged();
        emit roiSelected(m_rois.selectedRoiId());
    }
    markOverlayDirty();
    update();
    return true;
}

void VisionDisplayItem::addLine(const QString& id, double x1, double y1, double x2, double y2)
{
    GraphicObject graphic;
    graphic.id = id;
    graphic.type = GraphicType::Line;
    graphic.layer = defaultGraphicLayer;
    graphic.line = QLineF(QPointF(x1, y1), QPointF(x2, y2));
    m_graphics.addGraphic(graphic);
    markOverlayDirty();
    update();
}

void VisionDisplayItem::addRect(const QString& id, double x, double y, double w, double h)
{
    GraphicObject graphic;
    graphic.id = id;
    graphic.type = GraphicType::Rect;
    graphic.layer = defaultGraphicLayer;
    graphic.rect = QRectF(x, y, w, h);
    m_graphics.addGraphic(graphic);
    markOverlayDirty();
    update();
}

void VisionDisplayItem::addCircle(const QString& id, double cx, double cy, double r)
{
    GraphicObject graphic;
    graphic.id = id;
    graphic.type = GraphicType::Circle;
    graphic.layer = defaultGraphicLayer;
    graphic.center = QPointF(cx, cy);
    graphic.radius = r;
    m_graphics.addGraphic(graphic);
    markOverlayDirty();
    update();
}

void VisionDisplayItem::addText(const QString& id, double x, double y, const QString& text)
{
    GraphicObject graphic;
    graphic.id = id;
    graphic.type = GraphicType::Text;
    graphic.layer = defaultGraphicLayer;
    graphic.textPosition = QPointF(x, y);
    graphic.text = text;
    m_graphics.addGraphic(graphic);
    markOverlayDirty();
    update();
}

void VisionDisplayItem::addCross(const QString& id, double x, double y, double size)
{
    GraphicObject graphic;
    graphic.id = id;
    graphic.type = GraphicType::Cross;
    graphic.layer = defaultGraphicLayer;
    graphic.crossCenter = QPointF(x, y);
    graphic.crossSize = size;
    m_graphics.addGraphic(graphic);
    markOverlayDirty();
    update();
}

void VisionDisplayItem::addPolyline(const QString& id, const QVariantList& points)
{
    GraphicObject graphic;
    graphic.id = id;
    graphic.type = GraphicType::Polyline;
    graphic.layer = defaultGraphicLayer;
    graphic.points = pointsFromVariantList(points);
    m_graphics.addGraphic(graphic);
    markOverlayDirty();
    update();
}

void VisionDisplayItem::addDefectBox(const QString& id, double x, double y, double w, double h, const QString& label)
{
    GraphicObject graphic;
    graphic.id = id;
    graphic.type = GraphicType::DefectBox;
    graphic.layer = static_cast<int>(DisplayLayer::Result);
    graphic.resultGraphic = true;
    graphic.rect = QRectF(x, y, w, h);
    graphic.label = label;
    graphic.style = resultStyle(QColor(255, 80, 80), 2.0, 15);
    addResultGraphic(graphic);
}

void VisionDisplayItem::addDefectContour(const QString& id, const QVariantList& points, const QString& label)
{
    GraphicObject graphic;
    graphic.id = id;
    graphic.type = GraphicType::DefectContour;
    graphic.layer = static_cast<int>(DisplayLayer::Result);
    graphic.resultGraphic = true;
    graphic.points = pointsFromVariantList(points);
    graphic.label = label;
    graphic.style = resultStyle(QColor(255, 60, 60), 2.0, 15);
    addResultGraphic(graphic);
}

void VisionDisplayItem::addBlobRegion(const QString& id, const QVariantList& points)
{
    GraphicObject graphic;
    graphic.id = id;
    graphic.type = GraphicType::BlobRegion;
    graphic.layer = static_cast<int>(DisplayLayer::Result);
    graphic.resultGraphic = true;
    graphic.points = pointsFromVariantList(points);
    graphic.style = resultStyle(QColor(255, 190, 40), 2.0, 14);
    addResultGraphic(graphic);
}

void VisionDisplayItem::addMatchContour(const QString& id, const QVariantList& points)
{
    GraphicObject graphic;
    graphic.id = id;
    graphic.type = GraphicType::MatchContour;
    graphic.layer = static_cast<int>(DisplayLayer::Result);
    graphic.resultGraphic = true;
    graphic.points = pointsFromVariantList(points);
    graphic.style = resultStyle(QColor(80, 220, 120), 2.0, 14);
    addResultGraphic(graphic);
}

void VisionDisplayItem::addFittedLine(const QString& id, double x1, double y1, double x2, double y2)
{
    GraphicObject graphic;
    graphic.id = id;
    graphic.type = GraphicType::FittedLine;
    graphic.layer = static_cast<int>(DisplayLayer::Result);
    graphic.resultGraphic = true;
    graphic.line = QLineF(QPointF(x1, y1), QPointF(x2, y2));
    graphic.style = resultStyle(QColor(80, 180, 255), 2.0, 14);
    addResultGraphic(graphic);
}

void VisionDisplayItem::addFittedCircle(const QString& id, double cx, double cy, double r)
{
    GraphicObject graphic;
    graphic.id = id;
    graphic.type = GraphicType::FittedCircle;
    graphic.layer = static_cast<int>(DisplayLayer::Result);
    graphic.resultGraphic = true;
    graphic.center = QPointF(cx, cy);
    graphic.radius = r;
    graphic.style = resultStyle(QColor(80, 180, 255), 2.0, 14);
    addResultGraphic(graphic);
}

void VisionDisplayItem::setInspectionStatus(bool ok, const QString& text)
{
    GraphicObject graphic;
    graphic.id = QStringLiteral("__inspection_status__");
    graphic.type = GraphicType::StatusText;
    graphic.layer = static_cast<int>(DisplayLayer::Result);
    graphic.resultGraphic = true;
    graphic.statusOk = ok;
    graphic.textPosition = QPointF(24.0, 32.0);
    graphic.text = text;
    graphic.style = resultStyle(ok ? QColor(30, 220, 90) : QColor(255, 60, 60), 1.5, 28);
    addResultGraphic(graphic);
}

void VisionDisplayItem::addResultGraphics(const QVariantList& graphics)
{
    for (const QVariant& value : graphics) {
        const QVariantMap map = value.toMap();
        const QString type = map.value(QStringLiteral("type")).toString();
        const QString id = map.value(QStringLiteral("id")).toString();

        if (type == QStringLiteral("DefectBox")) {
            GraphicObject graphic;
            graphic.id = id;
            graphic.type = GraphicType::DefectBox;
            graphic.layer = static_cast<int>(DisplayLayer::Result);
            graphic.resultGraphic = true;
            graphic.rect = QRectF(map.value(QStringLiteral("x")).toDouble(),
                                  map.value(QStringLiteral("y")).toDouble(),
                                  map.value(QStringLiteral("w")).toDouble(),
                                  map.value(QStringLiteral("h")).toDouble());
            graphic.label = map.value(QStringLiteral("label")).toString();
            graphic.style = resultStyle(QColor(255, 80, 80), 2.0, 15);
            addResultGraphic(graphic, false);
        } else if (type == QStringLiteral("DefectContour")) {
            GraphicObject graphic;
            graphic.id = id;
            graphic.type = GraphicType::DefectContour;
            graphic.layer = static_cast<int>(DisplayLayer::Result);
            graphic.resultGraphic = true;
            graphic.points = pointsFromVariantList(map.value(QStringLiteral("points")).toList());
            graphic.label = map.value(QStringLiteral("label")).toString();
            graphic.style = resultStyle(QColor(255, 60, 60), 2.0, 15);
            addResultGraphic(graphic, false);
        } else if (type == QStringLiteral("BlobRegion") || type == QStringLiteral("MatchContour")) {
            GraphicObject graphic;
            graphic.id = id;
            graphic.type = type == QStringLiteral("BlobRegion") ? GraphicType::BlobRegion : GraphicType::MatchContour;
            graphic.layer = static_cast<int>(DisplayLayer::Result);
            graphic.resultGraphic = true;
            graphic.points = pointsFromVariantList(map.value(QStringLiteral("points")).toList());
            graphic.style = resultStyle(type == QStringLiteral("BlobRegion") ? QColor(255, 190, 40) : QColor(80, 220, 120), 2.0, 14);
            addResultGraphic(graphic, false);
        } else if (type == QStringLiteral("FittedLine")) {
            GraphicObject graphic;
            graphic.id = id;
            graphic.type = GraphicType::FittedLine;
            graphic.layer = static_cast<int>(DisplayLayer::Result);
            graphic.resultGraphic = true;
            graphic.line = QLineF(QPointF(map.value(QStringLiteral("x1")).toDouble(),
                                          map.value(QStringLiteral("y1")).toDouble()),
                                  QPointF(map.value(QStringLiteral("x2")).toDouble(),
                                          map.value(QStringLiteral("y2")).toDouble()));
            graphic.style = resultStyle(QColor(80, 180, 255), 2.0, 14);
            addResultGraphic(graphic, false);
        } else if (type == QStringLiteral("FittedCircle")) {
            GraphicObject graphic;
            graphic.id = id;
            graphic.type = GraphicType::FittedCircle;
            graphic.layer = static_cast<int>(DisplayLayer::Result);
            graphic.resultGraphic = true;
            graphic.center = QPointF(map.value(QStringLiteral("cx")).toDouble(),
                                     map.value(QStringLiteral("cy")).toDouble());
            graphic.radius = map.value(QStringLiteral("r")).toDouble();
            graphic.style = resultStyle(QColor(80, 180, 255), 2.0, 14);
            addResultGraphic(graphic, false);
        } else if (type == QStringLiteral("StatusText")) {
            const bool ok = map.value(QStringLiteral("ok")).toBool();
            GraphicObject graphic;
            graphic.id = id.isEmpty() ? QStringLiteral("__inspection_status__") : id;
            graphic.type = GraphicType::StatusText;
            graphic.layer = static_cast<int>(DisplayLayer::Result);
            graphic.resultGraphic = true;
            graphic.statusOk = ok;
            graphic.textPosition = QPointF(map.value(QStringLiteral("x"), 24.0).toDouble(),
                                           map.value(QStringLiteral("y"), 32.0).toDouble());
            graphic.text = map.value(QStringLiteral("text")).toString();
            graphic.style = resultStyle(ok ? QColor(30, 220, 90) : QColor(255, 60, 60), 1.5, 28);
            addResultGraphic(graphic, false);
        }
    }

    markOverlayDirty();
    update();
}

void VisionDisplayItem::clearToolGraphics(const QString& toolId)
{
    removeEditableCalipers(toolId);
    m_graphics.clearToolGraphics(toolId);
    markOverlayDirty();
    update();
}

void VisionDisplayItem::clearAllToolGraphics()
{
    m_editableCalipers.clear();
    m_editableCaliperDrag = EditableCaliperDrag();
    m_graphics.clearAllToolGraphics();
    markOverlayDirty();
    update();
}

void VisionDisplayItem::setToolGraphicsVisible(const QString& toolId, bool visible)
{
    m_graphics.setToolGraphicsVisible(toolId, visible);
    markOverlayDirty();
    update();
}

void VisionDisplayItem::setOverlayData(const QString& overlayId, const VisionDisplayOverlayData& data)
{
    if (overlayId.isEmpty()) {
        return;
    }

    m_graphics.clearToolGraphics(overlayId);
    int itemIndex = 0;

    for (const OverlayPolyline& polyline : data.polylines) {
        if (polyline.points.size() < 2) {
            continue;
        }
        GraphicObject graphic = baseToolGraphic(overlayId,
                                                QStringLiteral("%1_polyline_%2").arg(overlayId).arg(itemIndex++),
                                                QStringLiteral("Overlay"),
                                                GraphicType::Polyline,
                                                polyline.color);
        graphic.layer = static_cast<int>(DisplayLayer::Debug);
        graphic.points = polyline.points;
        graphic.style.lineWidth = std::max(1.0, polyline.width);
        addToolGraphic(graphic, false);

        if (!polyline.label.isEmpty()) {
            GraphicObject label = baseToolGraphic(overlayId,
                                                  QStringLiteral("%1_polyline_label_%2").arg(overlayId).arg(itemIndex++),
                                                  QStringLiteral("Overlay"),
                                                  GraphicType::Text,
                                                  polyline.color);
            label.layer = static_cast<int>(DisplayLayer::Debug);
            label.text = polyline.label;
            label.textPosition = polyline.points[polyline.points.size() / 2];
            label.style.fontPixelSize = 12;
            addToolGraphic(label, false);
        }
    }

    for (const OverlayArrow& arrow : data.arrows) {
        GraphicObject graphic = baseToolGraphic(overlayId,
                                                QStringLiteral("%1_arrow_%2").arg(overlayId).arg(itemIndex++),
                                                QStringLiteral("Overlay"),
                                                GraphicType::Arrow,
                                                arrow.color);
        graphic.layer = static_cast<int>(DisplayLayer::Debug);
        graphic.line = QLineF(arrow.p0, arrow.p1);
        graphic.style.lineWidth = std::max(1.0, arrow.width);
        graphic.arrowHeadSize = 7.0;
        addToolGraphic(graphic, false);
    }

    for (const OverlayPoint& point : data.points) {
        GraphicObject marker = baseToolGraphic(overlayId,
                                               QStringLiteral("%1_point_%2").arg(overlayId).arg(itemIndex++),
                                               QStringLiteral("Overlay"),
                                               GraphicType::PointMarker,
                                               point.color);
        marker.layer = static_cast<int>(DisplayLayer::Debug);
        marker.center = point.pos;
        marker.markerSize = std::max(2.0, point.radius * 2.0);
        marker.style.lineWidth = 2.0;
        addToolGraphic(marker, false);

        if (!point.label.isEmpty()) {
            GraphicObject label = baseToolGraphic(overlayId,
                                                  QStringLiteral("%1_point_label_%2").arg(overlayId).arg(itemIndex++),
                                                  QStringLiteral("Overlay"),
                                                  GraphicType::Text,
                                                  point.color);
            label.layer = static_cast<int>(DisplayLayer::Debug);
            label.textPosition = point.pos + QPointF(point.radius + 2.0, -point.radius - 2.0);
            label.text = point.label;
            label.style.fontPixelSize = 11;
            addToolGraphic(label, false);
        }
    }

    for (const OverlayText& text : data.texts) {
        if (text.text.isEmpty()) {
            continue;
        }
        GraphicObject graphic = baseToolGraphic(overlayId,
                                                QStringLiteral("%1_text_%2").arg(overlayId).arg(itemIndex++),
                                                QStringLiteral("Overlay"),
                                                GraphicType::Text,
                                                text.color);
        graphic.layer = static_cast<int>(DisplayLayer::Debug);
        graphic.textPosition = text.pos;
        graphic.text = text.text;
        graphic.style.fontPixelSize = std::max(1, text.fontSize);
        addToolGraphic(graphic, false);
    }

    markOverlayDirty();
    update();
}

void VisionDisplayItem::clearOverlayData(const QString& overlayId)
{
    clearToolGraphics(overlayId);
}

void VisionDisplayItem::addFindLineSearchRegion(const QString& toolId, double centerX, double centerY, double width, double height, double angleDeg)
{
    const QPointF center(centerX, centerY);
    if (!isFinitePoint(center) || !isFiniteValue(width) || !isFiniteValue(height) || !isFiniteValue(angleDeg)
        || width <= 0.0 || height <= 0.0) {
        return;
    }
    GraphicObject region = baseToolGraphic(toolId, toolId + QStringLiteral("_search_region"), QStringLiteral("FindLine"), GraphicType::RotatedRect, QColor(80, 190, 255));
    region.center = center;
    region.rect = QRectF(0.0, 0.0, width, height);
    region.angleDeg = angleDeg;
    addToolGraphic(region);
}

void VisionDisplayItem::addLineCaliper(const QString& toolId, const QString& id, double centerX, double centerY, double width, double height, double angleDeg, double searchDirectionAngleDeg, bool found, double edgeX, double edgeY, double score)
{
    const QPointF center(centerX, centerY);
    if (id.isEmpty() || !isFinitePoint(center) || !isFiniteValue(width) || !isFiniteValue(height)
        || !isFiniteValue(angleDeg) || !isFiniteValue(searchDirectionAngleDeg)
        || width <= 0.0 || height <= 0.0) {
        return;
    }
    GraphicObject caliper = baseToolGraphic(toolId, toolId + QStringLiteral("_caliper_") + id, QStringLiteral("FindLine"), GraphicType::RotatedRect, found ? QColor(255, 210, 80) : QColor(130, 130, 130), found);
    caliper.center = center;
    caliper.rect = QRectF(0.0, 0.0, width, height);
    caliper.angleDeg = angleDeg;
    addToolGraphic(caliper, false);

    GraphicObject arrow = baseToolGraphic(toolId, caliper.id + QStringLiteral("_arrow"), QStringLiteral("FindLine"), GraphicType::Arrow, QColor(255, 210, 80), found);
    const double arrowLength = std::max(width, height) * 0.35;
    arrow.line = QLineF(center, directionPoint(center, searchDirectionAngleDeg, arrowLength));
    arrow.arrowHeadSize = 9.0;
    addToolGraphic(arrow, false);

    if (found) {
        const QPointF edgePoint(edgeX, edgeY);
        if (!isFinitePoint(edgePoint)) {
            markOverlayDirty();
            update();
            return;
        }
        GraphicObject edge = baseToolGraphic(toolId, caliper.id + QStringLiteral("_edge"), QStringLiteral("FindLine"), GraphicType::PointMarker, QColor(0, 255, 120), true);
        edge.center = edgePoint;
        edge.markerSize = 8.0 + std::clamp(score, 0.0, 1.0) * 3.0;
        addToolGraphic(edge, false);
    }

    markOverlayDirty();
    update();
}

void VisionDisplayItem::addLineCalipers(const QString& toolId, const QVariantList& calipers)
{
    for (const QVariant& value : calipers) {
        const QVariantMap item = value.toMap();
        const QString id = item.value(QStringLiteral("id")).toString();
        const QPointF center(item.value(QStringLiteral("centerX")).toDouble(),
                             item.value(QStringLiteral("centerY")).toDouble());
        const double width = item.value(QStringLiteral("width")).toDouble();
        const double height = item.value(QStringLiteral("height")).toDouble();
        const double angleDeg = item.value(QStringLiteral("angleDeg")).toDouble();
        const double searchDirectionAngleDeg = item.value(QStringLiteral("searchDirectionAngleDeg")).toDouble();
        const bool found = item.value(QStringLiteral("found")).toBool();
        const double score = item.value(QStringLiteral("score")).toDouble();
        if (id.isEmpty() || !isFinitePoint(center) || !isFiniteValue(width) || !isFiniteValue(height)
            || !isFiniteValue(angleDeg) || !isFiniteValue(searchDirectionAngleDeg)
            || width <= 0.0 || height <= 0.0) {
            continue;
        }

        GraphicObject caliper = baseToolGraphic(toolId, toolId + QStringLiteral("_caliper_") + id, QStringLiteral("FindLine"), GraphicType::RotatedRect, found ? QColor(255, 210, 80) : QColor(130, 130, 130), found);
        caliper.center = center;
        caliper.rect = QRectF(0.0, 0.0, width, height);
        caliper.angleDeg = angleDeg;
        addToolGraphic(caliper, false);

        GraphicObject arrow = baseToolGraphic(toolId, caliper.id + QStringLiteral("_arrow"), QStringLiteral("FindLine"), GraphicType::Arrow, QColor(255, 210, 80), found);
        arrow.line = QLineF(center, directionPoint(center, searchDirectionAngleDeg, std::max(width, height) * 0.35));
        arrow.arrowHeadSize = 9.0;
        addToolGraphic(arrow, false);

        if (found) {
            const QPointF edgePoint(item.value(QStringLiteral("edgeX")).toDouble(),
                                    item.value(QStringLiteral("edgeY")).toDouble());
            if (!isFinitePoint(edgePoint)) {
                continue;
            }
            GraphicObject edge = baseToolGraphic(toolId, caliper.id + QStringLiteral("_edge"), QStringLiteral("FindLine"), GraphicType::PointMarker, QColor(0, 255, 120), true);
            edge.center = edgePoint;
            edge.markerSize = 8.0 + std::clamp(score, 0.0, 1.0) * 3.0;
            addToolGraphic(edge, false);
        }
    }
    markOverlayDirty();
    update();
}

void VisionDisplayItem::addEdgePoint(const QString& toolId, const QString& id, double x, double y)
{
    const QPointF point(x, y);
    if (id.isEmpty() || !isFinitePoint(point)) {
        return;
    }
    GraphicObject edge = baseToolGraphic(toolId, toolId + QStringLiteral("_edge_") + id, QStringLiteral("Tool"), GraphicType::PointMarker, QColor(0, 255, 120));
    edge.center = point;
    edge.markerSize = 8.0;
    addToolGraphic(edge);
}

void VisionDisplayItem::addEdgePoints(const QString& toolId, const QVariantList& points)
{
    int index = 0;
    for (const QVariant& value : points) {
        const QVariantMap item = value.toMap();
        if (!item.value(QStringLiteral("valid"), true).toBool()) {
            ++index;
            continue;
        }
        const QPointF point(item.value(QStringLiteral("x")).toDouble(),
                            item.value(QStringLiteral("y")).toDouble());
        if (!isFinitePoint(point)) {
            ++index;
            continue;
        }
        const bool outlier = item.value(QStringLiteral("outlier"), false).toBool();
        const bool inlier = item.value(QStringLiteral("inlier"), !outlier).toBool();
        const QColor color = outlier ? QColor(255, 80, 80) : (inlier ? QColor(0, 255, 120) : QColor(255, 210, 80));
        const QString pointKind = outlier ? QStringLiteral("outlier") : (inlier ? QStringLiteral("inlier") : QStringLiteral("candidate"));
        GraphicObject edge = baseToolGraphic(toolId, QStringLiteral("%1_%2_edge_%3").arg(toolId, pointKind).arg(index), QStringLiteral("Tool"), GraphicType::PointMarker, color);
        edge.center = point;
        edge.markerSize = outlier ? 9.0 : 7.0;
        addToolGraphic(edge, false);

        const double nx = item.value(QStringLiteral("nx")).toDouble();
        const double ny = item.value(QStringLiteral("ny")).toDouble();
        if (isFiniteValue(nx) && isFiniteValue(ny) && (!qFuzzyIsNull(nx) || !qFuzzyIsNull(ny))) {
            GraphicObject normal = baseToolGraphic(toolId, QStringLiteral("%1_%2_edge_normal_%3").arg(toolId, pointKind).arg(index), QStringLiteral("Tool"), GraphicType::Arrow, QColor(80, 220, 255));
            const QPointF p = edge.center;
            normal.line = QLineF(p, p + QPointF(nx, ny) * 20.0);
            normal.arrowHeadSize = 7.0;
            addToolGraphic(normal, false);
        }
        ++index;
    }
    markOverlayDirty();
    update();
}

void VisionDisplayItem::addFittedLineResult(const QString& toolId, double x1, double y1, double x2, double y2, double angleDeg, double score, double rmsError, const QString& label)
{
    const QPointF p1(x1, y1);
    const QPointF p2(x2, y2);
    if (!isFinitePoint(p1) || !isFinitePoint(p2) || !isFiniteValue(angleDeg)
        || !isFiniteValue(score) || !isFiniteValue(rmsError)) {
        return;
    }
    const bool ok = score >= 0.5;
    GraphicObject line = baseToolGraphic(toolId, toolId + QStringLiteral("_fitted_line"), QStringLiteral("FindLine"), GraphicType::FittedLine, QColor(40, 210, 90), ok);
    line.line = QLineF(p1, p2);
    line.style.lineWidth = 2.5;
    addToolGraphic(line, false);

    addEdgePoint(toolId, QStringLiteral("line_p1"), x1, y1);
    addEdgePoint(toolId, QStringLiteral("line_p2"), x2, y2);

    const QPointF mid((x1 + x2) * 0.5, (y1 + y2) * 0.5);
    GraphicObject normal = baseToolGraphic(toolId, toolId + QStringLiteral("_line_normal"), QStringLiteral("FindLine"), GraphicType::Arrow, QColor(80, 220, 255), ok);
    normal.line = QLineF(mid, directionPoint(mid, angleDeg + 90.0, 45.0));
    normal.arrowHeadSize = 10.0;
    addToolGraphic(normal, false);

    GraphicObject text = baseToolGraphic(toolId, toolId + QStringLiteral("_line_text"), QStringLiteral("FindLine"), GraphicType::Text, ok ? QColor(230, 255, 230) : QColor(255, 90, 90), ok);
    text.textPosition = mid + QPointF(12.0, -28.0);
    text.text = QStringLiteral("%1 Angle=%2 deg Score=%3 RMS=%4")
                    .arg(label.isEmpty() ? QStringLiteral("Line") : label)
                    .arg(angleDeg, 0, 'f', 2)
                    .arg(score, 0, 'f', 2)
                    .arg(rmsError, 0, 'f', 3);
    text.style.fontPixelSize = 15;
    addToolGraphic(text);
}

void VisionDisplayItem::addExpectedCircle(const QString& toolId, double centerX, double centerY, double radius)
{
    addExpectedArc(toolId, centerX, centerY, radius, 0.0, 360.0);
}

void VisionDisplayItem::addExpectedArc(const QString& toolId, double centerX, double centerY, double radius, double startAngleDeg, double spanAngleDeg)
{
    const QPointF center(centerX, centerY);
    if (!isFinitePoint(center) || !isFiniteValue(radius) || !isFiniteValue(startAngleDeg) || !isFiniteValue(spanAngleDeg)
        || radius <= 0.0 || std::abs(spanAngleDeg) <= 0.0001) {
        return;
    }
    GraphicObject arc = baseToolGraphic(toolId, toolId + QStringLiteral("_expected_arc"), QStringLiteral("FindCircle"), GraphicType::Arc, QColor(110, 170, 255));
    arc.center = center;
    arc.radius = radius;
    arc.startAngleDeg = startAngleDeg;
    arc.spanAngleDeg = spanAngleDeg;
    addToolGraphic(arc);
}

void VisionDisplayItem::addCircleSearchAnnulus(const QString& toolId, double centerX, double centerY, double innerRadius, double outerRadius, double startAngleDeg, double spanAngleDeg, double searchDirectionAngleDeg)
{
    const QPointF center(centerX, centerY);
    GraphicObject inner = baseToolGraphic(toolId, toolId + QStringLiteral("_annulus_inner"), QStringLiteral("FindCircle"), GraphicType::Arc, QColor(80, 190, 255));
    inner.center = center;
    inner.radius = innerRadius;
    inner.startAngleDeg = startAngleDeg;
    inner.spanAngleDeg = spanAngleDeg;
    addToolGraphic(inner, false);

    GraphicObject outer = inner;
    outer.id = toolId + QStringLiteral("_annulus_outer");
    outer.radius = outerRadius;
    addToolGraphic(outer, false);

    const bool fullCircle = std::abs(std::abs(spanAngleDeg) - 360.0) < 0.001;
    if (!fullCircle) {
        GraphicObject start = baseToolGraphic(toolId, toolId + QStringLiteral("_annulus_start"), QStringLiteral("FindCircle"), GraphicType::Line, QColor(80, 190, 255));
        start.line = QLineF(directionPoint(center, startAngleDeg, innerRadius), directionPoint(center, startAngleDeg, outerRadius));
        addToolGraphic(start, false);
        GraphicObject end = start;
        end.id = toolId + QStringLiteral("_annulus_end");
        end.line = QLineF(directionPoint(center, startAngleDeg + spanAngleDeg, innerRadius), directionPoint(center, startAngleDeg + spanAngleDeg, outerRadius));
        addToolGraphic(end, false);
    }

    GraphicObject arrow = baseToolGraphic(toolId, toolId + QStringLiteral("_annulus_direction"), QStringLiteral("FindCircle"), GraphicType::Arrow, QColor(255, 210, 80));
    const double midRadius = (innerRadius + outerRadius) * 0.5;
    const double midAngle = startAngleDeg + spanAngleDeg * 0.5;
    const QPointF p = directionPoint(center, midAngle, midRadius);
    arrow.line = QLineF(p, directionPoint(p, searchDirectionAngleDeg, 40.0));
    addToolGraphic(arrow);
}

void VisionDisplayItem::addRadialCaliper(const QString& toolId, const QString& id, double centerX, double centerY, double width, double height, double angleDeg, double searchDirectionAngleDeg, bool found, double edgeX, double edgeY, double score)
{
    addLineCaliper(toolId, id, centerX, centerY, width, height, angleDeg, searchDirectionAngleDeg, found, edgeX, edgeY, score);
}

void VisionDisplayItem::addRadialCalipers(const QString& toolId, const QVariantList& calipers)
{
    addLineCalipers(toolId, calipers);
}

void VisionDisplayItem::addFittedCircleResult(const QString& toolId, double centerX, double centerY, double radius, double score, double rmsError, const QString& label)
{
    const bool ok = score >= 0.5;
    GraphicObject circle = baseToolGraphic(toolId, toolId + QStringLiteral("_fitted_circle"), QStringLiteral("FindCircle"), GraphicType::FittedCircle, QColor(40, 210, 90), ok);
    circle.center = QPointF(centerX, centerY);
    circle.radius = radius;
    circle.style.lineWidth = 2.5;
    addToolGraphic(circle, false);

    GraphicObject center = baseToolGraphic(toolId, toolId + QStringLiteral("_center_cross"), QStringLiteral("FindCircle"), GraphicType::PointMarker, QColor(40, 210, 90), ok);
    center.center = circle.center;
    center.markerSize = 13.0;
    addToolGraphic(center, false);

    GraphicObject text = baseToolGraphic(toolId, toolId + QStringLiteral("_circle_text"), QStringLiteral("FindCircle"), GraphicType::Text, ok ? QColor(230, 255, 230) : QColor(255, 90, 90), ok);
    text.textPosition = circle.center + QPointF(radius * 0.25, -radius * 0.25);
    text.text = QStringLiteral("%1 R=%2 D=%3 Score=%4 RMS=%5")
                    .arg(label.isEmpty() ? QStringLiteral("Circle") : label)
                    .arg(radius, 0, 'f', 2)
                    .arg(radius * 2.0, 0, 'f', 2)
                    .arg(score, 0, 'f', 2)
                    .arg(rmsError, 0, 'f', 3);
    text.style.fontPixelSize = 15;
    addToolGraphic(text);
}

void VisionDisplayItem::addExpectedEllipse(const QString& toolId,
                                           double centerX,
                                           double centerY,
                                           double radiusA,
                                           double radiusB,
                                           double angleDeg,
                                           double startAngleDeg,
                                           double spanAngleDeg)
{
    GraphicObject ellipse = baseToolGraphic(toolId, toolId + QStringLiteral("_expected_ellipse"), QStringLiteral("FindEllipse"), GraphicType::Polyline, QColor(110, 170, 255));
    ellipse.points = ellipsePoints(QPointF(centerX, centerY), radiusA, radiusB, angleDeg, startAngleDeg, spanAngleDeg);
    ellipse.style.lineWidth = 1.5;
    addToolGraphic(ellipse);
}

void VisionDisplayItem::addFittedEllipseResult(const QString& toolId,
                                               double centerX,
                                               double centerY,
                                               double radiusA,
                                               double radiusB,
                                               double angleDeg,
                                               double score,
                                               double rmsError,
                                               const QString& label)
{
    const bool ok = score >= 0.5;
    GraphicObject ellipse = baseToolGraphic(toolId, toolId + QStringLiteral("_fitted_ellipse"), QStringLiteral("FindEllipse"), GraphicType::Polyline, ok ? QColor(40, 255, 120) : QColor(255, 80, 80), ok);
    ellipse.points = ellipsePoints(QPointF(centerX, centerY), radiusA, radiusB, angleDeg, 0.0, 360.0);
    ellipse.style.lineWidth = 2.5;
    addToolGraphic(ellipse, false);

    const double theta = angleDeg * pi / 180.0;
    const QPointF center(centerX, centerY);
    const QPointF major(std::cos(theta) * radiusA, std::sin(theta) * radiusA);
    const QPointF minor(-std::sin(theta) * radiusB, std::cos(theta) * radiusB);

    GraphicObject majorAxis = baseToolGraphic(toolId, toolId + QStringLiteral("_major_axis"), QStringLiteral("FindEllipse"), GraphicType::Line, QColor(40, 255, 120), ok);
    majorAxis.line = QLineF(center - major, center + major);
    addToolGraphic(majorAxis, false);

    GraphicObject minorAxis = baseToolGraphic(toolId, toolId + QStringLiteral("_minor_axis"), QStringLiteral("FindEllipse"), GraphicType::Line, QColor(80, 220, 255), ok);
    minorAxis.line = QLineF(center - minor, center + minor);
    addToolGraphic(minorAxis, false);

    GraphicObject centerMarker = baseToolGraphic(toolId, toolId + QStringLiteral("_center"), QStringLiteral("FindEllipse"), GraphicType::PointMarker, QColor(255, 255, 80), ok);
    centerMarker.center = center;
    centerMarker.markerSize = 11.0;
    addToolGraphic(centerMarker, false);

    GraphicObject text = baseToolGraphic(toolId, toolId + QStringLiteral("_ellipse_text"), QStringLiteral("FindEllipse"), GraphicType::Text, ok ? QColor(230, 255, 230) : QColor(255, 90, 90), ok);
    text.textPosition = center + QPointF(radiusA * 0.25, -radiusB * 0.75);
    text.text = QStringLiteral("%1 A=%2 B=%3 Angle=%4 Score=%5 RMS=%6")
                    .arg(label.isEmpty() ? QStringLiteral("Ellipse") : label)
                    .arg(radiusA, 0, 'f', 2)
                    .arg(radiusB, 0, 'f', 2)
                    .arg(angleDeg, 0, 'f', 2)
                    .arg(score, 0, 'f', 2)
                    .arg(rmsError, 0, 'f', 3);
    text.style.fontPixelSize = 15;
    addToolGraphic(text);
}

void VisionDisplayItem::addToolPointMarker(const QString& toolId, const QString& id, double x, double y, int red, int green, int blue, double markerSize)
{
    GraphicObject marker = baseToolGraphic(toolId,
                                           toolId + QStringLiteral("_marker_") + id,
                                           QStringLiteral("Tool"),
                                           GraphicType::PointMarker,
                                           QColor(std::clamp(red, 0, 255), std::clamp(green, 0, 255), std::clamp(blue, 0, 255)));
    marker.center = QPointF(x, y);
    marker.markerSize = markerSize;
    marker.style.lineWidth = 2.5;
    addToolGraphic(marker);
}

void VisionDisplayItem::addToolRect(const QString& toolId, const QString& id, double x, double y, double width, double height, int red, int green, int blue)
{
    if (!isFiniteValue(x) || !isFiniteValue(y) || !isFiniteValue(width) || !isFiniteValue(height)
        || width <= 0.0 || height <= 0.0) {
        return;
    }

    GraphicObject rect = baseToolGraphic(toolId,
                                         toolId + QStringLiteral("_rect_") + id,
                                         QStringLiteral("ToolRect"),
                                         GraphicType::Rect,
                                         QColor(std::clamp(red, 0, 255),
                                                std::clamp(green, 0, 255),
                                                std::clamp(blue, 0, 255)));
    rect.rect = QRectF(x, y, width, height);
    addToolGraphic(rect);
}

void VisionDisplayItem::addToolLine(const QString& toolId, const QString& id, double x1, double y1, double x2, double y2, int red, int green, int blue)
{
    if (!isFiniteValue(x1) || !isFiniteValue(y1) || !isFiniteValue(x2) || !isFiniteValue(y2)) {
        return;
    }

    GraphicObject line = baseToolGraphic(toolId,
                                         toolId + QStringLiteral("_line_") + id,
                                         QStringLiteral("ToolLine"),
                                         GraphicType::Line,
                                         QColor(std::clamp(red, 0, 255),
                                                std::clamp(green, 0, 255),
                                                std::clamp(blue, 0, 255)));
    line.line = QLineF(QPointF(x1, y1), QPointF(x2, y2));
    addToolGraphic(line);
}

void VisionDisplayItem::addCaliperResult(const QString& toolId, double centerX, double centerY, double width, double height, double angleDeg, double searchDirectionAngleDeg, const QVariantList& edgePoints, int bestIndex, double score, const QString& label)
{
    addLineCaliper(toolId, QStringLiteral("main"), centerX, centerY, width, height, angleDeg, searchDirectionAngleDeg, bestIndex >= 0, centerX, centerY, score);
    int index = 0;
    for (const QVariant& value : edgePoints) {
        QPointF p;
        if (value.canConvert<QPointF>()) {
            p = value.toPointF();
        } else {
            const QVariantMap map = value.toMap();
            p = QPointF(map.value(QStringLiteral("x")).toDouble(), map.value(QStringLiteral("y")).toDouble());
        }
        if (!isFinitePoint(p)) {
            ++index;
            continue;
        }
        GraphicObject edge = baseToolGraphic(toolId, QStringLiteral("%1_candidate_%2").arg(toolId).arg(index), QStringLiteral("Caliper"), GraphicType::PointMarker, index == bestIndex ? QColor(40, 255, 90) : QColor(255, 210, 80), true);
        edge.center = p;
        edge.markerSize = index == bestIndex ? 12.0 : 7.0;
        addToolGraphic(edge, false);
        ++index;
    }
    addToolStatusText(toolId, centerX + width * 0.5, centerY - height * 0.5, QStringLiteral("%1 Score=%2").arg(label).arg(score, 0, 'f', 2), score >= 0.5);
}

void VisionDisplayItem::addCaliperRegion(const QString& toolId,
                                         double centerX,
                                         double centerY,
                                         double width,
                                         double height,
                                         double angleDeg,
                                         double searchDirectionAngleDeg,
                                         const QString& label)
{
    GraphicObject caliper = baseToolGraphic(toolId,
                                            toolId + QStringLiteral("_caliper_region"),
                                            QStringLiteral("Caliper"),
                                            GraphicType::RotatedRect,
                                            QColor(255, 210, 80),
                                            true);
    caliper.center = QPointF(centerX, centerY);
    caliper.rect = QRectF(0.0, 0.0, width, height);
    caliper.angleDeg = angleDeg;
    addToolGraphic(caliper, false);

    GraphicObject searchArrow = baseToolGraphic(toolId,
                                                toolId + QStringLiteral("_search_arrow"),
                                                QStringLiteral("Caliper"),
                                                GraphicType::Arrow,
                                                QColor(80, 220, 255),
                                                true);
    const QPointF center(centerX, centerY);
    searchArrow.line = QLineF(center, directionPoint(center, searchDirectionAngleDeg, std::max(width, height) * 0.42));
    searchArrow.arrowHeadSize = 9.0;
    addToolGraphic(searchArrow, false);

    if (!label.isEmpty()) {
        addToolStatusText(toolId, centerX + width * 0.5, centerY - height * 0.5, label, true);
    } else {
        markOverlayDirty();
        update();
    }
}

void VisionDisplayItem::addCaliperEdgePoints(const QString& toolId,
                                             const QVariantList& edgePoints,
                                             int selectedIndex,
                                             const QString& label,
                                             bool showCandidateLabels)
{
    int index = 0;
    QPointF selectedPoint;
    bool hasSelected = false;

    for (const QVariant& value : edgePoints) {
        QPointF point;
        QVariantMap map;
        if (value.canConvert<QPointF>()) {
            point = value.toPointF();
        } else {
            map = value.toMap();
            point = QPointF(map.value(QStringLiteral("x")).toDouble(),
                            map.value(QStringLiteral("y")).toDouble());
        }
        if (!std::isfinite(point.x()) || !std::isfinite(point.y())) {
            ++index;
            continue;
        }

        const bool selected = index == selectedIndex;
        const bool withinExpected = map.value(QStringLiteral("withinExpected"), true).toBool();
        const QColor markerColor = selected
            ? QColor(40, 255, 90)
            : (withinExpected ? QColor(255, 210, 80) : QColor(255, 80, 80));
        GraphicObject marker = baseToolGraphic(toolId,
                                               QStringLiteral("%1_caliper_edge_%2").arg(toolId).arg(index),
                                               QStringLiteral("Caliper"),
                                               GraphicType::PointMarker,
                                               markerColor,
                                               selected || withinExpected);
        marker.center = point;
        marker.markerSize = selected ? 12.0 : (withinExpected ? 7.0 : 9.0);
        marker.style.lineWidth = selected ? 2.5 : 1.5;
        addToolGraphic(marker, false);

        if (showCandidateLabels) {
            const double response = map.value(QStringLiteral("response")).toDouble();
            const double position1D = map.value(QStringLiteral("position1D")).toDouble();
            GraphicObject text = baseToolGraphic(toolId,
                                                 QStringLiteral("%1_caliper_edge_label_%2").arg(toolId).arg(index),
                                                 QStringLiteral("Caliper"),
                                                 GraphicType::Text,
                                                 selected ? QColor(40, 255, 90) : (withinExpected ? QColor(255, 230, 120) : QColor(255, 120, 120)),
                                                 selected || withinExpected);
            text.textPosition = point + QPointF(8.0, selected ? -18.0 : 12.0);
            text.text = QStringLiteral("#%1 r=%2 p=%3 %4")
                            .arg(index)
                            .arg(response, 0, 'f', 2)
                            .arg(position1D, 0, 'f', 2)
                            .arg(withinExpected ? QStringLiteral("in") : QStringLiteral("out"));
            text.style.fontPixelSize = selected ? 14 : 11;
            addToolGraphic(text, false);
        }

        if (selected) {
            selectedPoint = point;
            hasSelected = true;
        }
        ++index;
    }

    if (!label.isEmpty() && hasSelected) {
        addToolStatusText(toolId, selectedPoint.x() + 12.0, selectedPoint.y() - 18.0, label, true);
    } else {
        markOverlayDirty();
        update();
    }
}

void VisionDisplayItem::addSingleCaliperDebugOverlay(const QString& toolId,
                                                     double centerX,
                                                     double centerY,
                                                     double width,
                                                     double height,
                                                     double angleDeg,
                                                     double searchDirectionAngleDeg,
                                                     const QVariantList& edgePoints,
                                                     int selectedIndex,
                                                     const QString& label)
{
    clearToolGraphics(toolId);

    const QPointF center(centerX, centerY);
    GraphicObject caliper = baseToolGraphic(toolId, toolId + QStringLiteral("_caliper"), QStringLiteral("SingleCaliper"), GraphicType::RotatedRect, QColor(255, 210, 80), true);
    caliper.center = center;
    caliper.rect = QRectF(0.0, 0.0, width, height);
    caliper.angleDeg = angleDeg;
    addToolGraphic(caliper, false);

    GraphicObject searchArrow = baseToolGraphic(toolId, toolId + QStringLiteral("_search_arrow"), QStringLiteral("SingleCaliper"), GraphicType::Arrow, QColor(80, 220, 255), true);
    searchArrow.line = QLineF(center, directionPoint(center, searchDirectionAngleDeg, std::max(width, height) * 0.42));
    searchArrow.arrowHeadSize = 9.0;
    addToolGraphic(searchArrow, false);

    int index = 0;
    QPointF selectedPoint;
    QPointF selectedNormal;
    double selectedResponse = 0.0;
    double selectedPosition1D = 0.0;
    bool hasSelected = false;

    for (const QVariant& value : edgePoints) {
        const QVariantMap item = value.toMap();
        const QPointF point(item.value(QStringLiteral("x")).toDouble(),
                            item.value(QStringLiteral("y")).toDouble());
        if (!isFinitePoint(point)) {
            ++index;
            continue;
        }
        const bool selected = index == selectedIndex;
        GraphicObject marker = baseToolGraphic(toolId,
                                               QStringLiteral("%1_candidate_%2").arg(toolId).arg(index),
                                               QStringLiteral("SingleCaliper"),
                                               GraphicType::PointMarker,
                                               selected ? QColor(40, 255, 90) : QColor(255, 210, 80),
                                               true);
        marker.center = point;
        marker.markerSize = selected ? 14.0 : 7.0;
        marker.style.lineWidth = selected ? 2.5 : 1.5;
        addToolGraphic(marker, false);

        if (selected) {
            const QPointF normal(item.value(QStringLiteral("nx")).toDouble(),
                                 item.value(QStringLiteral("ny")).toDouble());
            const double response = item.value(QStringLiteral("response")).toDouble();
            const double position1D = item.value(QStringLiteral("position1D")).toDouble();
            if (isFinitePoint(normal) && isFiniteValue(response) && isFiniteValue(position1D)) {
                selectedPoint = point;
                selectedNormal = normal;
                selectedResponse = response;
                selectedPosition1D = position1D;
                hasSelected = true;
            }
        }
        ++index;
    }

    if (hasSelected && isFiniteValue(height)) {
        const double normalLength = std::max(18.0, height * 0.18);
        GraphicObject normal = baseToolGraphic(toolId, toolId + QStringLiteral("_selected_normal"), QStringLiteral("SingleCaliper"), GraphicType::Arrow, QColor(40, 255, 90), true);
        normal.line = QLineF(selectedPoint, selectedPoint + selectedNormal * normalLength);
        normal.arrowHeadSize = 8.0;
        addToolGraphic(normal, false);

        GraphicObject text = baseToolGraphic(toolId, toolId + QStringLiteral("_selected_text"), QStringLiteral("SingleCaliper"), GraphicType::Text, QColor(230, 255, 230), true);
        text.textPosition = selectedPoint + QPointF(12.0, -18.0);
        text.text = QStringLiteral("response=%1 pos1D=%2")
                        .arg(selectedResponse, 0, 'f', 3)
                        .arg(selectedPosition1D, 0, 'f', 3);
        text.style.fontPixelSize = 14;
        text.layer = static_cast<int>(DisplayLayer::Debug);
        addToolGraphic(text, false);
    }

    GraphicObject summary = baseToolGraphic(toolId, toolId + QStringLiteral("_summary_text"), QStringLiteral("SingleCaliper"), GraphicType::Text, hasSelected ? QColor(40, 255, 100) : QColor(255, 80, 80), hasSelected);
    summary.textPosition = QPointF(centerX + width * 0.5, centerY - height * 0.5);
    summary.text = QStringLiteral("%1 candidates=%2").arg(label).arg(edgePoints.size());
    summary.style.fontPixelSize = 14;
    summary.layer = static_cast<int>(DisplayLayer::Debug);
    addToolGraphic(summary, false);

    markOverlayDirty();
    update();
}

void VisionDisplayItem::addDistanceResult(const QString& toolId, double x1, double y1, double x2, double y2, double distanceValue, const QString& unit, const QString& label)
{
    const bool ok = distanceValue >= 0.0;
    GraphicObject line = baseToolGraphic(toolId, toolId + QStringLiteral("_distance_line"), QStringLiteral("Distance"), GraphicType::Arrow, QColor(80, 220, 255), ok);
    line.line = QLineF(QPointF(x1, y1), QPointF(x2, y2));
    addToolGraphic(line, false);
    addEdgePoint(toolId, QStringLiteral("distance_p1"), x1, y1);
    addEdgePoint(toolId, QStringLiteral("distance_p2"), x2, y2);
    const QPointF mid((x1 + x2) * 0.5, (y1 + y2) * 0.5);
    addToolStatusText(toolId, mid.x() + 10.0, mid.y() - 10.0, metricText(label.isEmpty() ? QStringLiteral("Distance") : label, distanceValue, unit), ok);
}

void VisionDisplayItem::addAngleResult(const QString& toolId, double cx, double cy, double x1, double y1, double x2, double y2, double angleDeg, const QString& label)
{
    const bool ok = angleDeg >= 0.0;
    GraphicObject arm1 = baseToolGraphic(toolId, toolId + QStringLiteral("_angle_arm1"), QStringLiteral("Angle"), GraphicType::Line, QColor(180, 140, 255), ok);
    arm1.line = QLineF(QPointF(cx, cy), QPointF(x1, y1));
    addToolGraphic(arm1, false);
    GraphicObject arm2 = arm1;
    arm2.id = toolId + QStringLiteral("_angle_arm2");
    arm2.line = QLineF(QPointF(cx, cy), QPointF(x2, y2));
    addToolGraphic(arm2, false);

    const double a1 = std::atan2(y1 - cy, x1 - cx) * 180.0 / pi;
    const double radius = std::min(QLineF(QPointF(cx, cy), QPointF(x1, y1)).length(),
                                   QLineF(QPointF(cx, cy), QPointF(x2, y2)).length()) * 0.35;
    GraphicObject arc = baseToolGraphic(toolId, toolId + QStringLiteral("_angle_arc"), QStringLiteral("Angle"), GraphicType::Arc, QColor(180, 140, 255), ok);
    arc.center = QPointF(cx, cy);
    arc.radius = radius;
    arc.startAngleDeg = a1;
    arc.spanAngleDeg = angleDeg;
    addToolGraphic(arc, false);

    addToolStatusText(toolId, cx + radius * 0.55, cy - radius * 0.25, QStringLiteral("%1 = %2 deg").arg(label.isEmpty() ? QStringLiteral("Angle") : label).arg(angleDeg, 0, 'f', 2), ok);
}

void VisionDisplayItem::addToolStatusText(const QString& toolId, double x, double y, const QString& text, bool ok)
{
    GraphicObject graphic = baseToolGraphic(toolId, QStringLiteral("%1_status_%2_%3").arg(toolId).arg(x).arg(y), QStringLiteral("ToolText"), GraphicType::Text, ok ? QColor(40, 255, 100) : QColor(255, 80, 80), ok);
    graphic.textPosition = QPointF(x, y);
    graphic.text = text;
    graphic.style.fontPixelSize = 15;
    graphic.layer = static_cast<int>(DisplayLayer::Debug);
    addToolGraphic(graphic);
}

void VisionDisplayItem::updateDemoFrame(int frameIndex)
{
    constexpr int demoWidth = 1280;
    constexpr int demoHeight = 800;

    QImage image(demoWidth, demoHeight, QImage::Format_ARGB32);
    for (int y = 0; y < demoHeight; ++y) {
        auto* line = reinterpret_cast<QRgb*>(image.scanLine(y));
        for (int x = 0; x < demoWidth; ++x) {
            const int band = ((x / 80) + (y / 80) + frameIndex / 3) % 2;
            const int base = band ? 68 : 38;
            const int red = std::min(255, base + ((x + frameIndex * 17) % demoWidth) * 150 / demoWidth);
            const int green = std::min(255, base + y * 175 / demoHeight);
            const int blue = std::min(255, 110 + ((x + y + frameIndex * 9) % (demoWidth + demoHeight)) * 85 / (demoWidth + demoHeight));
            line[x] = qRgba(red, green, blue, 255);
        }
    }

    updateFrame(image);
}

void VisionDisplayItem::createEditableLineCalipers(const QString& toolId, double x1, double y1, double x2, double y2, double caliperWidth, double caliperHeight, int count)
{
    removeEditableCalipers(toolId);
    EditableCaliperArray array;
    array.toolId = toolId;
    array.kind = EditableCaliperKind::Line;
    array.p1 = QPointF(x1, y1);
    array.p2 = QPointF(x2, y2);
    array.caliperWidth = std::max(1.0, caliperWidth);
    array.caliperHeight = std::max(1.0, caliperHeight);
    array.count = std::max(1, count);
    m_editableCalipers.append(array);
    rebuildEditableCalipers(toolId);
}

void VisionDisplayItem::createEditableCircleCalipers(const QString& toolId, double cx, double cy, double radius, double caliperWidth, double caliperHeight, int count)
{
    removeEditableCalipers(toolId);
    EditableCaliperArray array;
    array.toolId = toolId;
    array.kind = EditableCaliperKind::Circle;
    array.center = QPointF(cx, cy);
    array.radius = std::max(1.0, radius);
    array.caliperWidth = std::max(1.0, caliperWidth);
    array.caliperHeight = std::max(1.0, caliperHeight);
    array.count = std::max(3, count);
    m_editableCalipers.append(array);
    rebuildEditableCalipers(toolId);
}

void VisionDisplayItem::createEditableEllipseCalipers(const QString& toolId, double cx, double cy, double radiusX, double radiusY, double angleDeg, double caliperWidth, double caliperHeight, int count)
{
    removeEditableCalipers(toolId);
    EditableCaliperArray array;
    array.toolId = toolId;
    array.kind = EditableCaliperKind::Ellipse;
    array.center = QPointF(cx, cy);
    array.radiusX = std::max(1.0, radiusX);
    array.radiusY = std::max(1.0, radiusY);
    array.angleDeg = angleDeg;
    array.caliperWidth = std::max(1.0, caliperWidth);
    array.caliperHeight = std::max(1.0, caliperHeight);
    array.count = std::max(4, count);
    m_editableCalipers.append(array);
    rebuildEditableCalipers(toolId);
}

void VisionDisplayItem::createEditableSingleCaliper(const QString& toolId, double centerX, double centerY, double width, double height, double searchDirectionAngleDeg)
{
    removeEditableCalipers(toolId);
    EditableCaliperArray array;
    array.toolId = toolId;
    array.kind = EditableCaliperKind::Single;
    array.center = QPointF(centerX, centerY);
    array.caliperWidth = std::max(1.0, width);
    array.caliperHeight = std::max(1.0, height);
    array.angleDeg = searchDirectionAngleDeg;
    array.count = 1;
    m_editableCalipers.append(array);
    rebuildEditableCalipers(toolId);
}

QVariantMap VisionDisplayItem::editableCaliperGeometry(const QString& toolId) const
{
    QVariantMap geometry;
    const auto found = std::find_if(m_editableCalipers.cbegin(),
                                    m_editableCalipers.cend(),
                                    [&toolId](const EditableCaliperArray& item) {
                                        return item.toolId == toolId;
                                    });
    if (found == m_editableCalipers.cend()) {
        geometry.insert(QStringLiteral("valid"), false);
        return geometry;
    }

    geometry.insert(QStringLiteral("valid"), true);
    geometry.insert(QStringLiteral("toolId"), found->toolId);
    geometry.insert(QStringLiteral("caliperWidth"), found->caliperWidth);
    geometry.insert(QStringLiteral("caliperHeight"), found->caliperHeight);
    geometry.insert(QStringLiteral("count"), found->count);

    switch (found->kind) {
    case EditableCaliperKind::Line:
        geometry.insert(QStringLiteral("type"), QStringLiteral("line"));
        geometry.insert(QStringLiteral("x1"), found->p1.x());
        geometry.insert(QStringLiteral("y1"), found->p1.y());
        geometry.insert(QStringLiteral("x2"), found->p2.x());
        geometry.insert(QStringLiteral("y2"), found->p2.y());
        geometry.insert(QStringLiteral("centerX"), (found->p1.x() + found->p2.x()) * 0.5);
        geometry.insert(QStringLiteral("centerY"), (found->p1.y() + found->p2.y()) * 0.5);
        geometry.insert(QStringLiteral("length"), QLineF(found->p1, found->p2).length());
        geometry.insert(QStringLiteral("searchLength"), found->caliperHeight);
        geometry.insert(QStringLiteral("angleDeg"), vectorAngleDeg(found->p2 - found->p1));
        break;
    case EditableCaliperKind::Circle:
        geometry.insert(QStringLiteral("type"), QStringLiteral("circle"));
        geometry.insert(QStringLiteral("centerX"), found->center.x());
        geometry.insert(QStringLiteral("centerY"), found->center.y());
        geometry.insert(QStringLiteral("radius"), found->radius);
        break;
    case EditableCaliperKind::Ellipse:
        geometry.insert(QStringLiteral("type"), QStringLiteral("ellipse"));
        geometry.insert(QStringLiteral("centerX"), found->center.x());
        geometry.insert(QStringLiteral("centerY"), found->center.y());
        geometry.insert(QStringLiteral("radiusX"), found->radiusX);
        geometry.insert(QStringLiteral("radiusY"), found->radiusY);
        geometry.insert(QStringLiteral("angleDeg"), found->angleDeg);
        break;
    case EditableCaliperKind::Single:
        geometry.insert(QStringLiteral("type"), QStringLiteral("single"));
        geometry.insert(QStringLiteral("centerX"), found->center.x());
        geometry.insert(QStringLiteral("centerY"), found->center.y());
        geometry.insert(QStringLiteral("width"), found->caliperWidth);
        geometry.insert(QStringLiteral("height"), found->caliperHeight);
        geometry.insert(QStringLiteral("searchDirectionAngleDeg"), found->angleDeg);
        geometry.insert(QStringLiteral("caliperAngleDeg"), found->angleDeg - 90.0);
        break;
    }

    return geometry;
}

QSGNode* VisionDisplayItem::updatePaintNode(QSGNode* oldNode, UpdatePaintNodeData*)
{
    if (m_image.isNull() || !window()) {
        delete oldNode;
        return nullptr;
    }

    auto* rootNode = static_cast<DisplayRootNode*>(oldNode);
    if (!rootNode) {
        rootNode = new DisplayRootNode;
        m_textureDirty = true;
        m_overlayDirty = true;
    }

    if (m_textureDirty || !rootNode->imageTexture) {
        QSGTexture* texture = window()->createTextureFromImage(m_image);
        if (!texture) {
            return rootNode;
        }
        texture->setFiltering(QSGTexture::Linear);
        QSGTexture* oldTexture = rootNode->imageTexture;
        rootNode->imageTexture = texture;
        rootNode->imageNode->setTexture(texture);
        delete oldTexture;
        m_textureDirty = false;
    }

    rootNode->imageNode->setRect(imageViewRect());

    if (!m_overlayDirty) {
        return rootNode;
    }

    clearChildNodes(rootNode->overlayNode);
    if (m_modelRoiVisible && m_modelRoi.isValid()) {
        GraphicStyle modelStyle;
        modelStyle.strokeColor = QColor(255, 190, 40);
        modelStyle.lineWidth = 2.0;
        modelStyle.lineWidthInViewPixels = true;
        QSGNode* roiNode = createLineNode(mapPoints(rectPoints(m_modelRoi.normalized()), m_mapper),
                                          QSGGeometry::DrawLineStrip,
                                          modelStyle,
                                          m_mapper.zoom());
        if (roiNode) {
            rootNode->overlayNode->appendChildNode(roiNode);
        }

        if (m_modelRoiEditable) {
            GraphicStyle handleStyle = modelStyle;
            handleStyle.strokeColor = QColor(255, 240, 120);
            handleStyle.lineWidth = 2.5;
            const QVector<QPointF> handles = rectHandlePoints(m_modelRoi);
            for (const QPointF& handle : handles) {
                QSGNode* handleNode = createPointMarkerNode(m_mapper.imageToView(handle), handleStyle, 12.0);
                if (handleNode) {
                    rootNode->overlayNode->appendChildNode(handleNode);
                }
            }
        }
    }

    for (const std::unique_ptr<RoiObject>& roi : m_rois.rois()) {
        if (!roi || !roi->visible) {
            continue;
        }

        const QSGGeometry::DrawingMode drawingMode = roi->type == RoiType::Line
            ? QSGGeometry::DrawLines
            : QSGGeometry::DrawLineStrip;
        QSGNode* roiNode = createLineNode(mapPoints(roi->outlinePoints(), m_mapper),
                                          drawingMode,
                                          roiStyle(*roi),
                                          m_mapper.zoom());
        if (roiNode) {
            rootNode->overlayNode->appendChildNode(roiNode);
        }
        if (roi->selected && roi->type == RoiType::Rect) {
            GraphicStyle handleStyle = roiStyle(*roi);
            handleStyle.strokeColor = QColor(255, 210, 80);
            handleStyle.lineWidth = 2.0;
            const QRectF rect = static_cast<const RectRoi*>(roi.get())->rect;
            const QVector<QPointF> handles = rectHandlePoints(rect);
            for (const QPointF& handle : handles) {
                QSGNode* handleNode = createPointMarkerNode(m_mapper.imageToView(handle),
                                                            handleStyle,
                                                            11.0);
                if (handleNode) {
                    rootNode->overlayNode->appendChildNode(handleNode);
                }
            }
        }
    }

    for (const GraphicObject& graphic : m_graphics.graphics()) {
        if (!graphic.visible || !m_graphics.isLayerVisible(graphic.layer)) {
            continue;
        }
        if (graphic.toolGraphic && !m_graphics.isToolGraphicsVisible(graphic.toolId)) {
            continue;
        }

        QSGNode* graphicNode = nullptr;
        switch (graphic.type) {
        case GraphicType::Line:
            graphicNode = createLineNode({m_mapper.imageToView(graphic.line.p1()),
                                          m_mapper.imageToView(graphic.line.p2())},
                                         QSGGeometry::DrawLines,
                                         graphic.style,
                                         m_mapper.zoom());
            break;
        case GraphicType::Rect:
            graphicNode = createLineNode(mapPoints(rectPoints(graphic.rect), m_mapper),
                                         QSGGeometry::DrawLineStrip,
                                         graphic.style,
                                         m_mapper.zoom());
            break;
        case GraphicType::Circle:
            graphicNode = createLineNode(mapPoints(circlePoints(graphic.center, graphic.radius), m_mapper),
                                         QSGGeometry::DrawLineStrip,
                                         graphic.style,
                                         m_mapper.zoom());
            break;
        case GraphicType::Text:
            graphicNode = createTextNode(graphic, m_mapper, window(), m_mapper.zoom());
            break;
        case GraphicType::Cross:
            graphicNode = createLineNode(mapPoints(crossPoints(graphic.crossCenter, graphic.crossSize), m_mapper),
                                         QSGGeometry::DrawLines,
                                         graphic.style,
                                         m_mapper.zoom());
            break;
        case GraphicType::Polyline:
            graphicNode = createLineNode(mapPoints(graphic.points, m_mapper),
                                         QSGGeometry::DrawLineStrip,
                                         graphic.style,
                                         m_mapper.zoom());
            break;
        case GraphicType::DefectBox: {
            auto* group = new QSGNode;
            if (auto* boxNode = createLineNode(mapPoints(rectPoints(graphic.rect), m_mapper),
                                               QSGGeometry::DrawLineStrip,
                                               graphic.style,
                                               m_mapper.zoom())) {
                group->appendChildNode(boxNode);
            }
            if (auto* labelNode = createGraphicLabelNode(graphic,
                                                         graphic.rect.topLeft() + QPointF(0.0, -18.0 / m_mapper.zoom()),
                                                         m_mapper,
                                                         window())) {
                group->appendChildNode(labelNode);
            }
            graphicNode = group;
            break;
        }
        case GraphicType::DefectContour: {
            auto* group = new QSGNode;
            if (auto* contourNode = createLineNode(mapPoints(closedPoints(graphic.points), m_mapper),
                                                   QSGGeometry::DrawLineStrip,
                                                   graphic.style,
                                                   m_mapper.zoom())) {
                group->appendChildNode(contourNode);
            }
            if (!graphic.points.isEmpty()) {
                if (auto* labelNode = createGraphicLabelNode(graphic, graphic.points.first(), m_mapper, window())) {
                    group->appendChildNode(labelNode);
                }
            }
            graphicNode = group;
            break;
        }
        case GraphicType::BlobRegion:
            graphicNode = createLineNode(mapPoints(closedPoints(graphic.points), m_mapper),
                                         QSGGeometry::DrawLineStrip,
                                         graphic.style,
                                         m_mapper.zoom());
            break;
        case GraphicType::MatchContour:
            graphicNode = createLineNode(mapPoints(graphic.points, m_mapper),
                                         QSGGeometry::DrawLineStrip,
                                         graphic.style,
                                         m_mapper.zoom());
            break;
        case GraphicType::FittedLine:
            graphicNode = createLineNode({m_mapper.imageToView(graphic.line.p1()),
                                          m_mapper.imageToView(graphic.line.p2())},
                                         QSGGeometry::DrawLines,
                                         graphic.style,
                                         m_mapper.zoom());
            break;
        case GraphicType::FittedCircle:
            graphicNode = createLineNode(mapPoints(circlePoints(graphic.center, graphic.radius), m_mapper),
                                         QSGGeometry::DrawLineStrip,
                                         graphic.style,
                                         m_mapper.zoom());
            break;
        case GraphicType::StatusText:
            graphicNode = createTextNode(graphic, m_mapper, window(), m_mapper.zoom());
            break;
        case GraphicType::RotatedRect:
            graphicNode = createLineNode(mapPoints(rotatedRectPoints(graphic.center,
                                                                      graphic.rect.width(),
                                                                      graphic.rect.height(),
                                                                      graphic.angleDeg),
                                                   m_mapper),
                                         QSGGeometry::DrawLineStrip,
                                         graphic.style,
                                         m_mapper.zoom());
            break;
        case GraphicType::Arc:
            graphicNode = createLineNode(mapPoints(arcPoints(graphic.center,
                                                             graphic.radius,
                                                             graphic.startAngleDeg,
                                                             graphic.spanAngleDeg),
                                                   m_mapper),
                                         QSGGeometry::DrawLineStrip,
                                         graphic.style,
                                         m_mapper.zoom());
            break;
        case GraphicType::Arrow:
            graphicNode = createArrowNode(graphic.line, m_mapper, graphic.style, graphic.arrowHeadSize);
            break;
        case GraphicType::PointMarker:
            graphicNode = createPointMarkerNode(m_mapper.imageToView(graphic.center),
                                                graphic.style,
                                                graphic.markerSize);
            break;
        }

        if (graphicNode) {
            rootNode->overlayNode->appendChildNode(graphicNode);
        }
    }

    m_overlayDirty = false;
    return rootNode;
}

void VisionDisplayItem::mousePressEvent(QMouseEvent* event)
{
    forceActiveFocus(Qt::MouseFocusReason);
    emitMouseImagePosition(event->position());

    if (event->button() == Qt::RightButton) {
        const QPointF imagePoint = m_mapper.viewToImage(event->position());
        emit imageRightClicked(imagePoint.x(), imagePoint.y());
        event->accept();
        return;
    }

    if (event->button() == Qt::LeftButton) {
        const QPointF imagePoint = m_mapper.viewToImage(event->position());
        if (beginModelRoiDrag(imagePoint)) {
            event->accept();
            return;
        }

        if (RoiObject* selected = m_rois.selectedRoi()) {
            if (selected->type == RoiType::Rect && !selected->locked) {
                const QRectF rect = static_cast<RectRoi*>(selected)->rect;
                const int handle = rectHandleAt(rect, imagePoint, imageHitTolerance(8.0));
                if (handle >= 0) {
                    m_draggingRoi = beginRoiDrag(selected->id, imagePoint);
                    m_roiDragSnapshot.handle = handle;
                    event->accept();
                    return;
                }
            }
        }
        if (beginEditableCaliperDrag(imagePoint)) {
            event->accept();
            return;
        }

        const QString hitRoiId = m_rois.hitTest(imagePoint, imageHitTolerance());
        if (!hitRoiId.isEmpty()) {
            selectRoi(hitRoiId);
            m_draggingRoi = beginRoiDrag(hitRoiId, imagePoint);
            event->accept();
            return;
        }
    }

    if ((event->button() == Qt::LeftButton || event->button() == Qt::MiddleButton)
        && m_interactionMode == PanMode) {
        m_panning = true;
        m_lastPanPoint = event->position();
        event->accept();
        return;
    }

    QQuickItem::mousePressEvent(event);
}

void VisionDisplayItem::mouseMoveEvent(QMouseEvent* event)
{
    emitMouseImagePosition(event->position());

    if (m_editableCaliperDrag.active) {
        if (updateEditableCaliperDrag(m_mapper.viewToImage(event->position()))) {
            markOverlayDirty();
            update();
        }
        event->accept();
        return;
    }

    if (m_modelRoiDrag.active) {
        if (updateModelRoiDrag(m_mapper.viewToImage(event->position()))) {
            markOverlayDirty();
            update();
        }
        event->accept();
        return;
    }

    if (m_draggingRoi) {
        if (!m_rois.selectedRoi() || m_rois.selectedRoi()->locked) {
            m_draggingRoi = false;
            m_roiDragSnapshot.valid = false;
            event->accept();
            return;
        }

        const QPointF currentImagePoint = m_mapper.viewToImage(event->position());
        if (updateDraggedRoi(currentImagePoint)) {
            emit roiChanged(m_roiDragSnapshot.id);
            markOverlayDirty();
            update();
        }
        event->accept();
        return;
    }

    if (m_panning) {
        const QPointF delta = event->position() - m_lastPanPoint;
        const QPointF nextOffset = m_mapper.offset() + delta;
        m_mapper.setOffset(nextOffset.x(), nextOffset.y());
        m_lastPanPoint = event->position();
        markOverlayDirty();
        update();
        event->accept();
        return;
    }

    QQuickItem::mouseMoveEvent(event);
}

void VisionDisplayItem::mouseReleaseEvent(QMouseEvent* event)
{
    emitMouseImagePosition(event->position());

    if (m_draggingRoi && event->button() == Qt::LeftButton) {
        m_draggingRoi = false;
        m_roiDragSnapshot.valid = false;
        event->accept();
        return;
    }

    if (m_editableCaliperDrag.active && event->button() == Qt::LeftButton) {
        m_editableCaliperDrag = EditableCaliperDrag();
        event->accept();
        return;
    }

    if (m_modelRoiDrag.active && event->button() == Qt::LeftButton) {
        m_modelRoiDrag = ModelRoiDrag();
        event->accept();
        return;
    }

    if (m_panning && (event->button() == Qt::LeftButton || event->button() == Qt::MiddleButton)) {
        m_panning = false;
        const QPointF imagePoint = m_mapper.viewToImage(event->position());
        emit imageClicked(imagePoint.x(), imagePoint.y());
        event->accept();
        return;
    }

    QQuickItem::mouseReleaseEvent(event);
}

void VisionDisplayItem::mouseDoubleClickEvent(QMouseEvent* event)
{
    const QPointF imagePoint = m_mapper.viewToImage(event->position());
    emit imageDoubleClicked(imagePoint.x(), imagePoint.y());

    if (event->button() == Qt::LeftButton) {
        if (m_modelRoiEditable) {
            resetModelRoi();
            event->accept();
            return;
        }
        fitToWindow();
        event->accept();
        return;
    }

    QQuickItem::mouseDoubleClickEvent(event);
}

void VisionDisplayItem::hoverMoveEvent(QHoverEvent* event)
{
    emitMouseImagePosition(event->position());
    updateModelRoiCursor(m_mapper.viewToImage(event->position()));
    QQuickItem::hoverMoveEvent(event);
}

void VisionDisplayItem::keyPressEvent(QKeyEvent* event)
{
    if (event->key() == Qt::Key_Delete) {
        deleteSelectedRoi();
        event->accept();
        return;
    }

    QQuickItem::keyPressEvent(event);
}

void VisionDisplayItem::wheelEvent(QWheelEvent* event)
{
    const double angleY = event->angleDelta().y();
    if (qFuzzyIsNull(angleY)) {
        QQuickItem::wheelEvent(event);
        return;
    }

    const double factor = std::pow(1.0015, angleY);
    setZoomAt(m_mapper.zoom() * factor, event->position().x(), event->position().y());
    emitMouseImagePosition(event->position());
    event->accept();
}

void VisionDisplayItem::geometryChange(const QRectF& newGeometry, const QRectF& oldGeometry)
{
    QQuickItem::geometryChange(newGeometry, oldGeometry);
    updateMapperViewSize();

    if (oldGeometry.size().isEmpty() && !newGeometry.size().isEmpty()) {
        fitToWindow();
    }
}

double VisionDisplayItem::boundedZoom(double zoom) const
{
    return std::clamp(zoom, m_minZoom, m_maxZoom);
}

QRectF VisionDisplayItem::imageViewRect() const
{
    const QPointF topLeft = m_mapper.imageToView(QPointF(0.0, 0.0));
    return QRectF(topLeft,
                  QSizeF(m_image.width() * m_mapper.zoom(), m_image.height() * m_mapper.zoom()));
}

void VisionDisplayItem::updateMapperViewSize()
{
    m_mapper.setViewSize(width(), height());
}

void VisionDisplayItem::emitMouseImagePosition(const QPointF& viewPoint)
{
    const QPointF imagePoint = m_mapper.viewToImage(viewPoint);
    emit mouseImagePositionChanged(imagePoint.x(), imagePoint.y());
}

double VisionDisplayItem::imageHitTolerance(double viewTolerance) const
{
    return viewTolerance / std::max(0.000001, m_mapper.zoom());
}

void VisionDisplayItem::selectRoi(const QString& id)
{
    if (!m_rois.selectRoi(id)) {
        return;
    }

    emit selectedRoiIdChanged();
    emit roiSelected(m_rois.selectedRoiId());
    markOverlayDirty();
    update();
}

bool VisionDisplayItem::beginRoiDrag(const QString& id, const QPointF& pressImagePoint)
{
    RoiObject* roi = m_rois.findById(id);
    if (!roi || roi->locked) {
        m_roiDragSnapshot = RoiDragSnapshot();
        return false;
    }

    RoiDragSnapshot snapshot;
    snapshot.id = id;
    snapshot.pressImagePoint = pressImagePoint;
    snapshot.type = roi->type;
    snapshot.valid = true;

    switch (roi->type) {
    case RoiType::Rect:
        snapshot.rect = static_cast<RectRoi*>(roi)->rect;
        break;
    case RoiType::RotatedRect: {
        const auto* rotatedRect = static_cast<RotatedRectRoi*>(roi);
        snapshot.center = rotatedRect->center;
        snapshot.width = rotatedRect->width;
        snapshot.height = rotatedRect->height;
        snapshot.angleDeg = rotatedRect->angleDeg;
        break;
    }
    case RoiType::Circle: {
        const auto* circle = static_cast<CircleRoi*>(roi);
        snapshot.center = circle->center;
        snapshot.radius = circle->radius;
        break;
    }
    case RoiType::Line:
        snapshot.line = static_cast<LineRoi*>(roi)->line;
        break;
    }

    m_roiDragSnapshot = snapshot;
    return true;
}

bool VisionDisplayItem::updateDraggedRoi(const QPointF& currentImagePoint)
{
    if (!m_roiDragSnapshot.valid) {
        return false;
    }

    RoiObject* roi = m_rois.findById(m_roiDragSnapshot.id);
    if (!roi || roi->locked) {
        return false;
    }

    const QPointF imageDelta = currentImagePoint - m_roiDragSnapshot.pressImagePoint;

    switch (m_roiDragSnapshot.type) {
    case RoiType::Rect:
        static_cast<RectRoi*>(roi)->rect = m_roiDragSnapshot.handle >= 0
            ? resizedRectFromHandle(m_roiDragSnapshot.rect, m_roiDragSnapshot.handle, currentImagePoint)
            : m_roiDragSnapshot.rect.translated(imageDelta);
        break;
    case RoiType::RotatedRect:
        static_cast<RotatedRectRoi*>(roi)->center = m_roiDragSnapshot.center + imageDelta;
        break;
    case RoiType::Circle:
        static_cast<CircleRoi*>(roi)->center = m_roiDragSnapshot.center + imageDelta;
        break;
    case RoiType::Line:
        static_cast<LineRoi*>(roi)->line = m_roiDragSnapshot.line.translated(imageDelta);
        break;
    }

    return true;
}

bool VisionDisplayItem::beginEditableCaliperDrag(const QPointF& imagePoint)
{
    const double tolerance = imageHitTolerance(9.0);
    for (const EditableCaliperArray& array : std::as_const(m_editableCalipers)) {
        QVector<QPointF> handles;
        switch (array.kind) {
        case EditableCaliperKind::Line: {
            const QLineF line(array.p1, array.p2);
            if (line.length() > 0.0001) {
                const QPointF axis = (array.p2 - array.p1) / line.length();
                const QPointF normal(-axis.y(), axis.x());
                const QPointF center = (array.p1 + array.p2) * 0.5;
                handles = {
                    array.p1,
                    array.p2,
                    center + normal * (array.caliperHeight * 0.5),
                    center - normal * (array.caliperHeight * 0.5)
                };

                for (qsizetype i = 0; i < handles.size(); ++i) {
                    if (pointDistance(imagePoint, handles[i]) <= tolerance) {
                        m_editableCaliperDrag.active = true;
                        m_editableCaliperDrag.toolId = array.toolId;
                        m_editableCaliperDrag.handle = static_cast<int>(i);
                        m_editableCaliperDrag.original = array;
                        m_editableCaliperDrag.pressImagePoint = imagePoint;
                        return true;
                    }
                }

                if (pointInRotatedRect(imagePoint,
                                       center,
                                       line.length(),
                                       array.caliperHeight,
                                       vectorAngleDeg(array.p2 - array.p1),
                                       tolerance)) {
                    m_editableCaliperDrag.active = true;
                    m_editableCaliperDrag.toolId = array.toolId;
                    m_editableCaliperDrag.handle = 4;
                    m_editableCaliperDrag.original = array;
                    m_editableCaliperDrag.pressImagePoint = imagePoint;
                    return true;
                }
            } else {
                handles = {array.p1, array.p2};
            }
            break;
        }
        case EditableCaliperKind::Circle: {
            const double halfSearchLength = array.caliperHeight * 0.5;
            const double innerRadius = std::max(1.0, array.radius - halfSearchLength);
            handles = {
                array.center,
                array.center + QPointF(array.radius, 0.0),
                array.center + QPointF(array.radius + halfSearchLength, 0.0),
                array.center + QPointF(innerRadius, 0.0)
            };

            for (qsizetype i = 0; i < handles.size(); ++i) {
                if (pointDistance(imagePoint, handles[i]) <= tolerance) {
                    m_editableCaliperDrag.active = true;
                    m_editableCaliperDrag.toolId = array.toolId;
                    m_editableCaliperDrag.handle = static_cast<int>(i);
                    m_editableCaliperDrag.original = array;
                    m_editableCaliperDrag.pressImagePoint = imagePoint;
                    return true;
                }
            }

            const double distance = pointDistance(imagePoint, array.center);
            if (distance >= innerRadius - tolerance
                && distance <= array.radius + halfSearchLength + tolerance) {
                m_editableCaliperDrag.active = true;
                m_editableCaliperDrag.toolId = array.toolId;
                m_editableCaliperDrag.handle = 4;
                m_editableCaliperDrag.original = array;
                m_editableCaliperDrag.pressImagePoint = imagePoint;
                return true;
            }
            break;
        }
        case EditableCaliperKind::Ellipse:
            handles = {array.center + rotatedVector(array.radiusX, 0.0, array.angleDeg),
                       array.center + rotatedVector(0.0, array.radiusY, array.angleDeg)};
            break;
        case EditableCaliperKind::Single: {
            const double caliperAngleDeg = array.angleDeg - 90.0;
            handles = {array.center,
                       directionPoint(array.center, array.angleDeg, array.caliperHeight * 0.62)};
            if (pointInRotatedRect(imagePoint,
                                   array.center,
                                   array.caliperWidth,
                                   array.caliperHeight,
                                   caliperAngleDeg,
                                   tolerance)) {
                m_editableCaliperDrag.active = true;
                m_editableCaliperDrag.toolId = array.toolId;
                m_editableCaliperDrag.handle = 0;
                m_editableCaliperDrag.original = array;
                m_editableCaliperDrag.pressImagePoint = imagePoint;
                return true;
            }
            break;
        }
        }

        for (qsizetype i = 0; i < handles.size(); ++i) {
            if (pointDistance(imagePoint, handles[i]) <= tolerance) {
                m_editableCaliperDrag.active = true;
                m_editableCaliperDrag.toolId = array.toolId;
                m_editableCaliperDrag.handle = static_cast<int>(i);
                m_editableCaliperDrag.original = array;
                m_editableCaliperDrag.pressImagePoint = imagePoint;
                return true;
            }
        }
    }
    return false;
}

bool VisionDisplayItem::updateEditableCaliperDrag(const QPointF& imagePoint)
{
    if (!m_editableCaliperDrag.active) {
        return false;
    }

    for (EditableCaliperArray& array : m_editableCalipers) {
        if (array.toolId != m_editableCaliperDrag.toolId) {
            continue;
        }

        const EditableCaliperArray& original = m_editableCaliperDrag.original;
        switch (array.kind) {
        case EditableCaliperKind::Line: {
            if (m_editableCaliperDrag.handle == 0) {
                array.p1 = imagePoint;
                array.p2 = original.p2;
            } else if (m_editableCaliperDrag.handle == 1) {
                array.p1 = original.p1;
                array.p2 = imagePoint;
            } else if (m_editableCaliperDrag.handle == 2 || m_editableCaliperDrag.handle == 3) {
                const QLineF line(original.p1, original.p2);
                if (line.length() > 0.0001) {
                    const QPointF axis = (original.p2 - original.p1) / line.length();
                    const QPointF normal(-axis.y(), axis.x());
                    const QPointF center = (original.p1 + original.p2) * 0.5;
                    const double signedDistance = QPointF::dotProduct(imagePoint - center, normal);
                    array.p1 = original.p1;
                    array.p2 = original.p2;
                    array.caliperHeight = std::max(1.0, std::abs(signedDistance) * 2.0);
                }
            } else {
                const QPointF delta = imagePoint - m_editableCaliperDrag.pressImagePoint;
                array.p1 = original.p1 + delta;
                array.p2 = original.p2 + delta;
            }
            break;
        }
        case EditableCaliperKind::Circle:
            if (m_editableCaliperDrag.handle == 0) {
                const QPointF delta = imagePoint - m_editableCaliperDrag.pressImagePoint;
                array.center = original.center + delta;
                array.radius = original.radius;
                array.caliperHeight = original.caliperHeight;
            } else if (m_editableCaliperDrag.handle == 1) {
                array.center = original.center;
                array.radius = std::max(1.0, pointDistance(original.center, imagePoint));
                array.caliperHeight = original.caliperHeight;
            } else if (m_editableCaliperDrag.handle == 2 || m_editableCaliperDrag.handle == 3) {
                array.center = original.center;
                array.radius = original.radius;
                array.caliperHeight = std::max(1.0, std::abs(pointDistance(original.center, imagePoint) - original.radius) * 2.0);
            } else {
                const QPointF delta = imagePoint - m_editableCaliperDrag.pressImagePoint;
                array.center = original.center + delta;
                array.radius = original.radius;
                array.caliperHeight = original.caliperHeight;
            }
            break;
        case EditableCaliperKind::Ellipse: {
            const QPointF delta = imagePoint - original.center;
            if (m_editableCaliperDrag.handle == 0) {
                const QPointF axis = rotatedVector(1.0, 0.0, original.angleDeg);
                array.radiusX = std::max(1.0, std::abs(QPointF::dotProduct(delta, axis)));
            } else {
                const QPointF axis = rotatedVector(0.0, 1.0, original.angleDeg);
                array.radiusY = std::max(1.0, std::abs(QPointF::dotProduct(delta, axis)));
            }
            break;
        }
        case EditableCaliperKind::Single:
            if (m_editableCaliperDrag.handle == 0) {
                const QPointF delta = imagePoint - m_editableCaliperDrag.pressImagePoint;
                array.center = original.center + delta;
            } else {
                array.center = original.center;
                array.angleDeg = vectorAngleDeg(imagePoint - original.center);
            }
            break;
        }

        rebuildEditableCalipers(array.toolId, false);
        return true;
    }
    return false;
}

int VisionDisplayItem::modelRoiHandleAt(const QPointF& imagePoint) const
{
    if (!m_modelRoiVisible || !m_modelRoiEditable || !m_modelRoi.isValid()) {
        return -1;
    }
    const double tolerance = imageHitTolerance(8.0);
    const QVector<QPointF> corners = rectHandlePoints(m_modelRoi);
    for (int i = 0; i < corners.size(); ++i) {
        if (QLineF(imagePoint, corners[i]).length() <= tolerance) {
            return i;
        }
    }
    const QVector<QPointF> edges = edgeHandlePoints(m_modelRoi);
    for (int i = 0; i < edges.size(); ++i) {
        if (QLineF(imagePoint, edges[i]).length() <= tolerance) {
            return 8 + i;
        }
    }
    if (m_modelRoi.normalized().contains(imagePoint)) {
        return 100;
    }
    return -1;
}

bool VisionDisplayItem::beginModelRoiDrag(const QPointF& imagePoint)
{
    const int handle = modelRoiHandleAt(imagePoint);
    if (handle < 0) {
        return false;
    }
    m_modelRoiDrag.active = true;
    m_modelRoiDrag.handle = handle;
    m_modelRoiDrag.originalRoi = m_modelRoi.normalized();
    m_modelRoiDrag.pressImagePoint = imagePoint;
    return true;
}

bool VisionDisplayItem::updateModelRoiDrag(const QPointF& imagePoint)
{
    if (!m_modelRoiDrag.active) {
        return false;
    }

    QRectF roi = m_modelRoiDrag.originalRoi.normalized();
    const QPointF delta = imagePoint - m_modelRoiDrag.pressImagePoint;
    if (m_modelRoiDrag.handle == 100) {
        roi.translate(delta);
    } else {
        double left = roi.left();
        double right = roi.right();
        double top = roi.top();
        double bottom = roi.bottom();
        switch (m_modelRoiDrag.handle) {
        case 0: left += delta.x(); top += delta.y(); break;
        case 1: top += delta.y(); break;
        case 2: right += delta.x(); top += delta.y(); break;
        case 3: right += delta.x(); break;
        case 4: right += delta.x(); bottom += delta.y(); break;
        case 5: bottom += delta.y(); break;
        case 6: left += delta.x(); bottom += delta.y(); break;
        case 7: left += delta.x(); break;
        case 8: top += delta.y(); break;
        case 9: right += delta.x(); break;
        case 10: bottom += delta.y(); break;
        case 11: left += delta.x(); break;
        default: break;
        }
        roi = QRectF(QPointF(left, top), QPointF(right, bottom)).normalized();
    }

    const QRectF next = boundedModelRoi(roi);
    if (m_modelRoi == next) {
        return false;
    }
    m_modelRoi = next;
    emit modelRoiChanged();
    return true;
}

QRectF VisionDisplayItem::boundedModelRoi(const QRectF& roi) const
{
    const QSize imageSize = m_mapper.imageSize();
    if (imageSize.isEmpty()) {
        return roi.normalized();
    }
    const QRectF imageRect(0.0, 0.0, imageSize.width(), imageSize.height());
    QRectF r = roi.normalized();
    if (r.width() < minModelRoiSize) {
        r.setWidth(minModelRoiSize);
    }
    if (r.height() < minModelRoiSize) {
        r.setHeight(minModelRoiSize);
    }
    if (r.width() > imageRect.width()) {
        r.setWidth(imageRect.width());
    }
    if (r.height() > imageRect.height()) {
        r.setHeight(imageRect.height());
    }
    if (r.left() < imageRect.left()) {
        r.moveLeft(imageRect.left());
    }
    if (r.top() < imageRect.top()) {
        r.moveTop(imageRect.top());
    }
    if (r.right() > imageRect.right()) {
        r.moveRight(imageRect.right());
    }
    if (r.bottom() > imageRect.bottom()) {
        r.moveBottom(imageRect.bottom());
    }
    return r.intersected(imageRect).normalized();
}

QRectF VisionDisplayItem::defaultModelRoi() const
{
    const QSize imageSize = m_mapper.imageSize();
    if (imageSize.isEmpty()) {
        return {};
    }
    const double w = std::max(minModelRoiSize, imageSize.width() * 0.6);
    const double h = std::max(minModelRoiSize, imageSize.height() * 0.6);
    return boundedModelRoi(QRectF((imageSize.width() - w) * 0.5,
                                  (imageSize.height() - h) * 0.5,
                                  w,
                                  h));
}

void VisionDisplayItem::updateModelRoiCursor(const QPointF& imagePoint)
{
    if (!m_modelRoiEditable || !m_modelRoiVisible || !m_modelRoi.isValid()) {
        unsetCursor();
        return;
    }
    const int handle = modelRoiHandleAt(imagePoint);
    switch (handle) {
    case 0:
    case 4:
        setCursor(Qt::SizeFDiagCursor);
        break;
    case 2:
    case 6:
        setCursor(Qt::SizeBDiagCursor);
        break;
    case 1:
    case 5:
    case 8:
    case 10:
        setCursor(Qt::SizeVerCursor);
        break;
    case 3:
    case 7:
    case 9:
    case 11:
        setCursor(Qt::SizeHorCursor);
        break;
    case 100:
        setCursor(Qt::SizeAllCursor);
        break;
    default:
        unsetCursor();
        break;
    }
}

void VisionDisplayItem::rebuildEditableCalipers(const QString& toolId, bool requestUpdate)
{
    const auto found = std::find_if(m_editableCalipers.begin(),
                                    m_editableCalipers.end(),
                                    [&toolId](const EditableCaliperArray& item) {
                                        return item.toolId == toolId;
                                    });
    if (found == m_editableCalipers.end()) {
        return;
    }

    const EditableCaliperArray array = *found;
    m_graphics.clearToolGraphics(toolId);

    if (array.kind == EditableCaliperKind::Line) {
        const QLineF line(array.p1, array.p2);
        const double length = line.length();
        const double angleDeg = vectorAngleDeg(array.p2 - array.p1);
        const QPointF center = (array.p1 + array.p2) * 0.5;
        const QPointF axis = length > 0.0001 ? (array.p2 - array.p1) / length : QPointF(1.0, 0.0);
        const QPointF normal(-axis.y(), axis.x());

        GraphicObject outline = baseToolGraphic(toolId, toolId + QStringLiteral("_editable_search_region"), QStringLiteral("EditableLineCalipers"), GraphicType::Polyline, QColor(80, 220, 255));
        const QPointF halfNormal = normal * (array.caliperHeight * 0.5);
        outline.points = {
            array.p1 + halfNormal,
            array.p2 + halfNormal,
            array.p2 - halfNormal,
            array.p1 - halfNormal,
            array.p1 + halfNormal
        };
        outline.style.lineWidth = 1.5;
        addToolGraphic(outline, false);

        GraphicObject baseline = baseToolGraphic(toolId, toolId + QStringLiteral("_editable_baseline"), QStringLiteral("EditableLineCalipers"), GraphicType::Line, QColor(80, 220, 255));
        baseline.line = line;
        baseline.style.lineWidth = 2.0;
        addToolGraphic(baseline, false);

        for (int i = 0; i < array.count; ++i) {
            const double t = (static_cast<double>(i) + 0.5) / array.count;
            const QPointF center = array.p1 + (array.p2 - array.p1) * t;
            GraphicObject caliper = baseToolGraphic(toolId, QStringLiteral("%1_edit_line_caliper_%2").arg(toolId).arg(i), QStringLiteral("EditableLineCalipers"), GraphicType::RotatedRect, QColor(255, 210, 80));
            caliper.center = center;
            caliper.rect = QRectF(0.0, 0.0, array.caliperWidth, array.caliperHeight);
            caliper.angleDeg = angleDeg;
            addToolGraphic(caliper, false);

            GraphicObject arrow = baseToolGraphic(toolId, caliper.id + QStringLiteral("_arrow"), QStringLiteral("EditableLineCalipers"), GraphicType::Arrow, QColor(255, 210, 80));
            arrow.line = QLineF(center, directionPoint(center, angleDeg + 90.0, array.caliperHeight * 0.35));
            arrow.arrowHeadSize = 9.0;
            addToolGraphic(arrow, false);
        }

        for (int i = 0; i < 2; ++i) {
            GraphicObject handle = baseToolGraphic(toolId, QStringLiteral("%1_line_handle_%2").arg(toolId).arg(i), QStringLiteral("EditableLineCalipers"), GraphicType::PointMarker, QColor(255, 80, 80));
            handle.center = i == 0 ? array.p1 : array.p2;
            handle.markerSize = 13.0;
            addToolGraphic(handle, false);
        }

        for (int i = 0; i < 2; ++i) {
            GraphicObject handle = baseToolGraphic(toolId, QStringLiteral("%1_line_height_handle_%2").arg(toolId).arg(i), QStringLiteral("EditableLineCalipers"), GraphicType::PointMarker, QColor(255, 170, 70));
            handle.center = i == 0 ? center + halfNormal : center - halfNormal;
            handle.markerSize = 11.0;
            addToolGraphic(handle, false);
        }

        GraphicObject moveHandle = baseToolGraphic(toolId, toolId + QStringLiteral("_line_move_handle"), QStringLiteral("EditableLineCalipers"), GraphicType::PointMarker, QColor(80, 220, 255));
        moveHandle.center = center;
        moveHandle.markerSize = 10.0;
        addToolGraphic(moveHandle, false);
    } else if (array.kind == EditableCaliperKind::Circle) {
        const double halfSearchLength = array.caliperHeight * 0.5;
        const double innerRadius = std::max(1.0, array.radius - halfSearchLength);
        const double outerRadius = array.radius + halfSearchLength;

        GraphicObject outerCircle = baseToolGraphic(toolId, toolId + QStringLiteral("_editable_outer_circle"), QStringLiteral("EditableCircleCalipers"), GraphicType::Arc, QColor(80, 220, 255));
        outerCircle.center = array.center;
        outerCircle.radius = outerRadius;
        outerCircle.startAngleDeg = 0.0;
        outerCircle.spanAngleDeg = 360.0;
        outerCircle.style.lineWidth = 1.3;
        addToolGraphic(outerCircle, false);

        GraphicObject innerCircle = baseToolGraphic(toolId, toolId + QStringLiteral("_editable_inner_circle"), QStringLiteral("EditableCircleCalipers"), GraphicType::Arc, QColor(80, 220, 255));
        innerCircle.center = array.center;
        innerCircle.radius = innerRadius;
        innerCircle.startAngleDeg = 0.0;
        innerCircle.spanAngleDeg = 360.0;
        innerCircle.style.lineWidth = 1.3;
        addToolGraphic(innerCircle, false);

        GraphicObject circle = baseToolGraphic(toolId, toolId + QStringLiteral("_editable_circle"), QStringLiteral("EditableCircleCalipers"), GraphicType::Arc, QColor(80, 220, 255));
        circle.center = array.center;
        circle.radius = array.radius;
        circle.startAngleDeg = 0.0;
        circle.spanAngleDeg = 360.0;
        addToolGraphic(circle, false);

        for (int i = 0; i < array.count; ++i) {
            const double thetaDeg = static_cast<double>(i) * 360.0 / array.count;
            const QPointF center = directionPoint(array.center, thetaDeg, array.radius);
            GraphicObject caliper = baseToolGraphic(toolId, QStringLiteral("%1_edit_circle_caliper_%2").arg(toolId).arg(i), QStringLiteral("EditableCircleCalipers"), GraphicType::RotatedRect, QColor(255, 210, 80));
            caliper.center = center;
            caliper.rect = QRectF(0.0, 0.0, array.caliperWidth, array.caliperHeight);
            caliper.angleDeg = thetaDeg + 90.0;
            addToolGraphic(caliper, false);

            GraphicObject arrow = baseToolGraphic(toolId, caliper.id + QStringLiteral("_arrow"), QStringLiteral("EditableCircleCalipers"), GraphicType::Arrow, QColor(255, 210, 80));
            arrow.line = QLineF(center, directionPoint(center, thetaDeg, array.caliperHeight * 0.35));
            arrow.arrowHeadSize = 9.0;
            addToolGraphic(arrow, false);
        }

        GraphicObject centerHandle = baseToolGraphic(toolId, toolId + QStringLiteral("_circle_center_handle"), QStringLiteral("EditableCircleCalipers"), GraphicType::PointMarker, QColor(80, 220, 255));
        centerHandle.center = array.center;
        centerHandle.markerSize = 11.0;
        addToolGraphic(centerHandle, false);

        GraphicObject radiusHandle = baseToolGraphic(toolId, toolId + QStringLiteral("_circle_radius_handle"), QStringLiteral("EditableCircleCalipers"), GraphicType::PointMarker, QColor(255, 80, 80));
        radiusHandle.center = array.center + QPointF(array.radius, 0.0);
        radiusHandle.markerSize = 13.0;
        addToolGraphic(radiusHandle, false);

        for (int i = 0; i < 2; ++i) {
            GraphicObject handle = baseToolGraphic(toolId, QStringLiteral("%1_circle_search_handle_%2").arg(toolId).arg(i), QStringLiteral("EditableCircleCalipers"), GraphicType::PointMarker, QColor(255, 170, 70));
            handle.center = array.center + QPointF(i == 0 ? outerRadius : innerRadius, 0.0);
            handle.markerSize = 11.0;
            addToolGraphic(handle, false);
        }
    } else if (array.kind == EditableCaliperKind::Ellipse) {
        QVector<QPointF> ellipse;
        ellipse.reserve(73);
        for (int i = 0; i <= 72; ++i) {
            const double theta = static_cast<double>(i) * 2.0 * pi / 72.0;
            ellipse.push_back(array.center + rotatedVector(std::cos(theta) * array.radiusX,
                                                           std::sin(theta) * array.radiusY,
                                                           array.angleDeg));
        }

        GraphicObject outline = baseToolGraphic(toolId, toolId + QStringLiteral("_editable_ellipse"), QStringLiteral("EditableEllipseCalipers"), GraphicType::Polyline, QColor(80, 220, 255));
        outline.points = ellipse;
        addToolGraphic(outline, false);

        for (int i = 0; i < array.count; ++i) {
            const double theta = static_cast<double>(i) * 2.0 * pi / array.count;
            const QPointF local(std::cos(theta) * array.radiusX, std::sin(theta) * array.radiusY);
            const QPointF center = array.center + rotatedVector(local.x(), local.y(), array.angleDeg);
            const QPointF tangent = rotatedVector(-array.radiusX * std::sin(theta), array.radiusY * std::cos(theta), array.angleDeg);
            const double angleDeg = vectorAngleDeg(tangent);

            GraphicObject caliper = baseToolGraphic(toolId, QStringLiteral("%1_edit_ellipse_caliper_%2").arg(toolId).arg(i), QStringLiteral("EditableEllipseCalipers"), GraphicType::RotatedRect, QColor(255, 210, 80));
            caliper.center = center;
            caliper.rect = QRectF(0.0, 0.0, array.caliperWidth, array.caliperHeight);
            caliper.angleDeg = angleDeg;
            addToolGraphic(caliper, false);

            GraphicObject arrow = baseToolGraphic(toolId, caliper.id + QStringLiteral("_arrow"), QStringLiteral("EditableEllipseCalipers"), GraphicType::Arrow, QColor(255, 210, 80));
            arrow.line = QLineF(center, array.center + (center - array.center) * 0.82);
            arrow.arrowHeadSize = 9.0;
            addToolGraphic(arrow, false);
        }

        const QPointF majorHandle = array.center + rotatedVector(array.radiusX, 0.0, array.angleDeg);
        const QPointF minorHandle = array.center + rotatedVector(0.0, array.radiusY, array.angleDeg);
        for (int i = 0; i < 2; ++i) {
            GraphicObject handle = baseToolGraphic(toolId, QStringLiteral("%1_ellipse_handle_%2").arg(toolId).arg(i), QStringLiteral("EditableEllipseCalipers"), GraphicType::PointMarker, QColor(255, 80, 80));
            handle.center = i == 0 ? majorHandle : minorHandle;
            handle.markerSize = 13.0;
            addToolGraphic(handle, false);
        }
    } else {
        const double caliperAngleDeg = array.angleDeg - 90.0;
        GraphicObject caliper = baseToolGraphic(toolId,
                                                toolId + QStringLiteral("_editable_single"),
                                                QStringLiteral("EditableSingleCaliper"),
                                                GraphicType::RotatedRect,
                                                QColor(255, 210, 80));
        caliper.center = array.center;
        caliper.rect = QRectF(0.0, 0.0, array.caliperWidth, array.caliperHeight);
        caliper.angleDeg = caliperAngleDeg;
        caliper.style.lineWidth = 2.0;
        addToolGraphic(caliper, false);

        GraphicObject searchArrow = baseToolGraphic(toolId,
                                                    toolId + QStringLiteral("_single_search_arrow"),
                                                    QStringLiteral("EditableSingleCaliper"),
                                                    GraphicType::Arrow,
                                                    QColor(80, 220, 255));
        searchArrow.line = QLineF(array.center,
                                  directionPoint(array.center, array.angleDeg, array.caliperHeight * 0.42));
        searchArrow.arrowHeadSize = 9.0;
        addToolGraphic(searchArrow, false);

        GraphicObject centerHandle = baseToolGraphic(toolId,
                                                     toolId + QStringLiteral("_single_center_handle"),
                                                     QStringLiteral("EditableSingleCaliper"),
                                                     GraphicType::PointMarker,
                                                     QColor(80, 220, 255));
        centerHandle.center = array.center;
        centerHandle.markerSize = 11.0;
        addToolGraphic(centerHandle, false);

        GraphicObject rotateHandle = baseToolGraphic(toolId,
                                                     toolId + QStringLiteral("_single_rotate_handle"),
                                                     QStringLiteral("EditableSingleCaliper"),
                                                     GraphicType::PointMarker,
                                                     QColor(255, 80, 80));
        rotateHandle.center = directionPoint(array.center, array.angleDeg, array.caliperHeight * 0.62);
        rotateHandle.markerSize = 13.0;
        addToolGraphic(rotateHandle, false);

        GraphicObject guide = baseToolGraphic(toolId,
                                              toolId + QStringLiteral("_single_rotate_guide"),
                                              QStringLiteral("EditableSingleCaliper"),
                                              GraphicType::Line,
                                              QColor(255, 80, 80));
        guide.line = QLineF(array.center, rotateHandle.center);
        guide.style.lineWidth = 1.0;
        addToolGraphic(guide, false);
    }

    if (requestUpdate) {
        markOverlayDirty();
        update();
    }
}

void VisionDisplayItem::removeEditableCalipers(const QString& toolId)
{
    for (auto it = m_editableCalipers.begin(); it != m_editableCalipers.end();) {
        if (it->toolId == toolId) {
            it = m_editableCalipers.erase(it);
        } else {
            ++it;
        }
    }
    if (m_editableCaliperDrag.toolId == toolId) {
        m_editableCaliperDrag = EditableCaliperDrag();
    }
}

QVector<QPointF> VisionDisplayItem::pointsFromVariantList(const QVariantList& points) const
{
    QVector<QPointF> result;
    result.reserve(points.size());

    for (const QVariant& pointValue : points) {
        if (pointValue.canConvert<QPointF>()) {
            const QPointF point = pointValue.toPointF();
            if (isFinitePoint(point)) {
                result.push_back(point);
            }
            continue;
        }

        const QVariantMap pointMap = pointValue.toMap();
        if (pointMap.contains(QStringLiteral("x")) && pointMap.contains(QStringLiteral("y"))) {
            const QPointF point(pointMap.value(QStringLiteral("x")).toDouble(),
                                pointMap.value(QStringLiteral("y")).toDouble());
            if (isFinitePoint(point)) {
                result.push_back(point);
            }
        }
    }

    return result;
}

void VisionDisplayItem::addResultGraphic(const GraphicObject& graphic, bool requestUpdate)
{
    if (graphic.id.isEmpty()) {
        return;
    }
    m_graphics.addGraphic(graphic);
    if (requestUpdate) {
        markOverlayDirty();
        update();
    }
}

void VisionDisplayItem::addToolGraphic(const GraphicObject& graphic, bool requestUpdate)
{
    if (graphic.id.isEmpty()) {
        return;
    }
    GraphicObject item = graphic;
    item.toolGraphic = true;
    if (item.layer == static_cast<int>(DisplayLayer::Result)) {
        item.layer = static_cast<int>(DisplayLayer::Measure);
    }
    m_graphics.addGraphic(item);
    if (requestUpdate) {
        markOverlayDirty();
        update();
    }
}

void VisionDisplayItem::applyFrame(FrameData frame)
{
    if (!frame.isValid()) {
        return;
    }

    const QSize oldSize = m_image.size();
    m_image = std::move(frame.image);
    ++m_frameSequence;
    m_mapper.setImageSize(m_image.width(), m_image.height());
    m_textureDirty = true;

    if (m_autoFitOnNewImage && !m_keepViewTransformOnNewImage && oldSize != m_image.size()) {
        fitToWindow();
        return;
    }

    update();
}

FrameData VisionDisplayItem::frameFromImage(const QImage& image)
{
    FrameData frame;
    if (image.isNull()) {
        return frame;
    }

    switch (image.format()) {
    case QImage::Format_ARGB32:
    case QImage::Format_ARGB32_Premultiplied:
    case QImage::Format_Grayscale8:
    case QImage::Format_RGB888:
    case QImage::Format_RGBA8888:
    case QImage::Format_RGBA8888_Premultiplied:
        frame.image = image.copy();
        break;
    default:
        frame.image = image.convertToFormat(QImage::Format_RGBA8888);
        break;
    }
    return frame;
}

FrameData VisionDisplayItem::frameFromRaw(const uchar* data, int width, int height, int stride, PixelFormat pixelFormat)
{
    FrameData frame;
    if (!data || width <= 0 || height <= 0) {
        return frame;
    }

    const int bpp = bytesPerPixel(pixelFormat);
    if (bpp <= 0) {
        return frame;
    }

    if (width > std::numeric_limits<int>::max() / bpp) {
        return frame;
    }
    const int minimumStride = width * bpp;
    if (stride <= 0) {
        stride = minimumStride;
    }
    if (stride < minimumStride) {
        return frame;
    }

    if (pixelFormat == PixelFormat::Gray8) {
        QImage image(width, height, QImage::Format_Grayscale8);
        for (int y = 0; y < height; ++y) {
            std::memcpy(image.scanLine(y), data + y * stride, static_cast<size_t>(width));
        }
        frame.image = std::move(image);
        return frame;
    }

    QImage image(width, height, QImage::Format_RGBA8888);
    for (int y = 0; y < height; ++y) {
        const uchar* src = data + y * stride;
        uchar* dst = image.scanLine(y);
        for (int x = 0; x < width; ++x) {
            const int si = x * bpp;
            const int di = x * 4;
            switch (pixelFormat) {
            case PixelFormat::RGB888:
                dst[di + 0] = src[si + 0];
                dst[di + 1] = src[si + 1];
                dst[di + 2] = src[si + 2];
                dst[di + 3] = 255;
                break;
            case PixelFormat::BGR888:
                dst[di + 0] = src[si + 2];
                dst[di + 1] = src[si + 1];
                dst[di + 2] = src[si + 0];
                dst[di + 3] = 255;
                break;
            case PixelFormat::RGBA8888:
                dst[di + 0] = src[si + 0];
                dst[di + 1] = src[si + 1];
                dst[di + 2] = src[si + 2];
                dst[di + 3] = src[si + 3];
                break;
            case PixelFormat::BGRA8888:
                dst[di + 0] = src[si + 2];
                dst[di + 1] = src[si + 1];
                dst[di + 2] = src[si + 0];
                dst[di + 3] = src[si + 3];
                break;
            case PixelFormat::Gray8:
                break;
            }
        }
    }

    frame.image = std::move(image);
    return frame;
}

void VisionDisplayItem::markOverlayDirty()
{
    m_overlayDirty = true;
}

} // namespace VisionDisplay
