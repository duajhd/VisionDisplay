#pragma once

#include "VisionDisplay/CoordinateMapper.h"
#include "VisionDisplay/FrameData.h"
#include "VisionDisplay/GraphicManager.h"
#include "VisionDisplay/OverlayData.h"
#include "VisionDisplay/RoiManager.h"
#include "VisionDisplay/VisionDisplayTypes.h"
#include "VisionDisplay/VisionDisplay_global.h"

#include <QImage>
#include <QLineF>
#include <QList>
#include <QPointF>
#include <QQuickItem>
#include <QRectF>
#include <QVariantList>
#include <QVariantMap>
#include <QtQml/qqmlregistration.h>

class QHoverEvent;
class QKeyEvent;
class QMouseEvent;
class QWheelEvent;
class QSGSimpleTextureNode;

namespace VisionDisplay {

class VISIONDISPLAY_API VisionDisplayItem : public QQuickItem
{
    Q_OBJECT
    QML_NAMED_ELEMENT(VisionDisplay)

    Q_PROPERTY(double zoom READ zoom WRITE setZoom NOTIFY zoomChanged)
    Q_PROPERTY(double minZoom READ minZoom WRITE setMinZoom NOTIFY minZoomChanged)
    Q_PROPERTY(double maxZoom READ maxZoom WRITE setMaxZoom NOTIFY maxZoomChanged)
    Q_PROPERTY(bool showPixelInfo READ showPixelInfo WRITE setShowPixelInfo NOTIFY showPixelInfoChanged)
    Q_PROPERTY(bool showCrosshair READ showCrosshair WRITE setShowCrosshair NOTIFY showCrosshairChanged)
    Q_PROPERTY(int interactionMode READ interactionMode WRITE setInteractionMode NOTIFY interactionModeChanged)
    Q_PROPERTY(QString selectedRoiId READ selectedRoiId NOTIFY selectedRoiIdChanged)
    Q_PROPERTY(bool autoFitOnNewImage READ autoFitOnNewImage WRITE setAutoFitOnNewImage NOTIFY autoFitOnNewImageChanged)
    Q_PROPERTY(bool keepViewTransformOnNewImage READ keepViewTransformOnNewImage WRITE setKeepViewTransformOnNewImage NOTIFY keepViewTransformOnNewImageChanged)
    Q_PROPERTY(QRectF modelRoi READ modelRoi WRITE setModelRoi NOTIFY modelRoiChanged)
    Q_PROPERTY(bool modelRoiVisible READ modelRoiVisible WRITE setModelRoiVisible NOTIFY modelRoiVisibleChanged)
    Q_PROPERTY(bool modelRoiEditable READ modelRoiEditable WRITE setModelRoiEditable NOTIFY modelRoiEditableChanged)

public:
    enum InteractionMode {
        NoInteraction = 0,
        PanMode = 1
    };
    Q_ENUM(InteractionMode)

    enum Layer {
        RoiLayer = static_cast<int>(DisplayLayer::Roi),
        ResultLayer = static_cast<int>(DisplayLayer::Result),
        MeasureLayer = static_cast<int>(DisplayLayer::Measure),
        TemporaryLayer = static_cast<int>(DisplayLayer::Temporary),
        DebugLayer = static_cast<int>(DisplayLayer::Debug)
    };
    Q_ENUM(Layer)

    explicit VisionDisplayItem(QQuickItem* parent = nullptr);
    ~VisionDisplayItem() override;

    double zoom() const;
    void setZoom(double zoom);

    double minZoom() const;
    void setMinZoom(double minZoom);

    double maxZoom() const;
    void setMaxZoom(double maxZoom);

    bool showPixelInfo() const;
    void setShowPixelInfo(bool showPixelInfo);

    bool showCrosshair() const;
    void setShowCrosshair(bool showCrosshair);

    int interactionMode() const;
    void setInteractionMode(int interactionMode);

    QString selectedRoiId() const;

    bool autoFitOnNewImage() const;
    Q_INVOKABLE void setAutoFitOnNewImage(bool enabled);

    bool keepViewTransformOnNewImage() const;
    Q_INVOKABLE void setKeepViewTransformOnNewImage(bool enabled);

    QRectF modelRoi() const;
    Q_INVOKABLE void setModelRoi(const QRectF& roi);
    bool modelRoiVisible() const;
    Q_INVOKABLE void setModelRoiVisible(bool visible);
    bool modelRoiEditable() const;
    Q_INVOKABLE void setModelRoiEditable(bool editable);
    Q_INVOKABLE void resetModelRoi();

    // Thread-safe frame entry points. These methods copy input pixels before returning
    // and use a queued call when invoked outside the item's owning thread.
    void setImage(const QImage& image);
    void updateFrame(const QImage& image);
    void updateFrame(const uchar* data, int width, int height, int stride, int pixelFormat);

    Q_INVOKABLE void fitToWindow();
    Q_INVOKABLE bool loadImage(const QString& pathOrUrl);
    Q_INVOKABLE void setZoomAt(double zoom, double viewX, double viewY);
    Q_INVOKABLE void zoomIn();
    Q_INVOKABLE void zoomOut();
    Q_INVOKABLE QPointF imageToView(const QPointF& imagePoint) const;
    Q_INVOKABLE QPointF viewToImage(const QPointF& viewPoint) const;

    Q_INVOKABLE void clearGraphics();
    Q_INVOKABLE void clearGraphicsByLayer(int layer);
    Q_INVOKABLE void clearResultGraphics();
    Q_INVOKABLE void setLayerVisible(int layer, bool visible);
    Q_INVOKABLE void clearRois();
    Q_INVOKABLE void createRectRoi(const QString& id, double x, double y, double w, double h);
    Q_INVOKABLE void createRotatedRectRoi(const QString& id, double cx, double cy, double w, double h, double angleDeg);
    Q_INVOKABLE void createCircleRoi(const QString& id, double cx, double cy, double r);
    Q_INVOKABLE void createLineRoi(const QString& id, double x1, double y1, double x2, double y2);
    Q_INVOKABLE void deleteSelectedRoi();
    Q_INVOKABLE QVariantMap roiGeometry(const QString& id) const;
    QString exportRoisJson() const;
    Q_INVOKABLE bool saveImage(const QString& path) const;
    Q_INVOKABLE bool saveScreenshot(const QString& path, bool withOverlay) const;
    Q_INVOKABLE bool exportGraphicsJson(const QString& path) const;
    Q_INVOKABLE bool exportRoisJson(const QString& path) const;
    Q_INVOKABLE bool importRoisJson(const QString& path);
    Q_INVOKABLE QString exportRoisJsonString() const;
    Q_INVOKABLE bool importRoisJsonString(const QString& json);
    Q_INVOKABLE void addLine(const QString& id, double x1, double y1, double x2, double y2);
    Q_INVOKABLE void addRect(const QString& id, double x, double y, double w, double h);
    Q_INVOKABLE void addCircle(const QString& id, double cx, double cy, double r);
    Q_INVOKABLE void addText(const QString& id, double x, double y, const QString& text);
    Q_INVOKABLE void addCross(const QString& id, double x, double y, double size);
    Q_INVOKABLE void addPolyline(const QString& id, const QVariantList& points);
    Q_INVOKABLE void addDefectBox(const QString& id, double x, double y, double w, double h, const QString& label);
    Q_INVOKABLE void addDefectContour(const QString& id, const QVariantList& points, const QString& label);
    Q_INVOKABLE void addBlobRegion(const QString& id, const QVariantList& points);
    Q_INVOKABLE void addMatchContour(const QString& id, const QVariantList& points);
    Q_INVOKABLE void addFittedLine(const QString& id, double x1, double y1, double x2, double y2);
    Q_INVOKABLE void addFittedCircle(const QString& id, double cx, double cy, double r);
    Q_INVOKABLE void setInspectionStatus(bool ok, const QString& text);
    Q_INVOKABLE void addResultGraphics(const QVariantList& graphics);
    Q_INVOKABLE void updateFrame(const QByteArray& data, int width, int height, int stride, int pixelFormat);
    Q_INVOKABLE void clearToolGraphics(const QString& toolId);
    Q_INVOKABLE void clearAllToolGraphics();
    Q_INVOKABLE void setToolGraphicsVisible(const QString& toolId, bool visible);
    void setOverlayData(const QString& overlayId, const VisionDisplayOverlayData& data);
    Q_INVOKABLE void clearOverlayData(const QString& overlayId);
    Q_INVOKABLE void addFindLineSearchRegion(const QString& toolId, double centerX, double centerY, double width, double height, double angleDeg);
    Q_INVOKABLE void addLineCaliper(const QString& toolId, const QString& id, double centerX, double centerY, double width, double height, double angleDeg, double searchDirectionAngleDeg, bool found, double edgeX, double edgeY, double score);
    Q_INVOKABLE void addLineCalipers(const QString& toolId, const QVariantList& calipers);
    Q_INVOKABLE void addEdgePoint(const QString& toolId, const QString& id, double x, double y);
    Q_INVOKABLE void addEdgePoints(const QString& toolId, const QVariantList& points);
    Q_INVOKABLE void addFittedLineResult(const QString& toolId, double x1, double y1, double x2, double y2, double angleDeg, double score, double rmsError, const QString& label);
    Q_INVOKABLE void addExpectedCircle(const QString& toolId, double centerX, double centerY, double radius);
    Q_INVOKABLE void addExpectedArc(const QString& toolId, double centerX, double centerY, double radius, double startAngleDeg, double spanAngleDeg);
    Q_INVOKABLE void addCircleSearchAnnulus(const QString& toolId, double centerX, double centerY, double innerRadius, double outerRadius, double startAngleDeg, double spanAngleDeg, double searchDirectionAngleDeg);
    Q_INVOKABLE void addRadialCaliper(const QString& toolId, const QString& id, double centerX, double centerY, double width, double height, double angleDeg, double searchDirectionAngleDeg, bool found, double edgeX, double edgeY, double score);
    Q_INVOKABLE void addRadialCalipers(const QString& toolId, const QVariantList& calipers);
    Q_INVOKABLE void addFittedCircleResult(const QString& toolId, double centerX, double centerY, double radius, double score, double rmsError, const QString& label);
    Q_INVOKABLE void addExpectedEllipse(const QString& toolId, double centerX, double centerY, double radiusA, double radiusB, double angleDeg, double startAngleDeg, double spanAngleDeg);
    Q_INVOKABLE void addFittedEllipseResult(const QString& toolId, double centerX, double centerY, double radiusA, double radiusB, double angleDeg, double score, double rmsError, const QString& label);
    Q_INVOKABLE void addToolPointMarker(const QString& toolId, const QString& id, double x, double y, int red, int green, int blue, double markerSize);
    Q_INVOKABLE void addToolRect(const QString& toolId, const QString& id, double x, double y, double width, double height, int red, int green, int blue);
    Q_INVOKABLE void addToolLine(const QString& toolId, const QString& id, double x1, double y1, double x2, double y2, int red, int green, int blue);
    Q_INVOKABLE void addCaliperResult(const QString& toolId, double centerX, double centerY, double width, double height, double angleDeg, double searchDirectionAngleDeg, const QVariantList& edgePoints, int bestIndex, double score, const QString& label);
    Q_INVOKABLE void addCaliperRegion(const QString& toolId, double centerX, double centerY, double width, double height, double angleDeg, double searchDirectionAngleDeg, const QString& label);
    Q_INVOKABLE void addCaliperEdgePoints(const QString& toolId, const QVariantList& edgePoints, int selectedIndex, const QString& label, bool showCandidateLabels);
    Q_INVOKABLE void addSingleCaliperDebugOverlay(const QString& toolId, double centerX, double centerY, double width, double height, double angleDeg, double searchDirectionAngleDeg, const QVariantList& edgePoints, int selectedIndex, const QString& label);
    Q_INVOKABLE void addDistanceResult(const QString& toolId, double x1, double y1, double x2, double y2, double distanceValue, const QString& unit, const QString& label);
    Q_INVOKABLE void addAngleResult(const QString& toolId, double cx, double cy, double x1, double y1, double x2, double y2, double angleDeg, const QString& label);
    Q_INVOKABLE void addToolStatusText(const QString& toolId, double x, double y, const QString& text, bool ok);
    Q_INVOKABLE void updateDemoFrame(int frameIndex);
    Q_INVOKABLE void createEditableLineCalipers(const QString& toolId, double x1, double y1, double x2, double y2, double caliperWidth, double caliperHeight, int count);
    Q_INVOKABLE void createEditableCircleCalipers(const QString& toolId, double cx, double cy, double radius, double caliperWidth, double caliperHeight, int count);
    Q_INVOKABLE void createEditableEllipseCalipers(const QString& toolId, double cx, double cy, double radiusX, double radiusY, double angleDeg, double caliperWidth, double caliperHeight, int count);
    Q_INVOKABLE void createEditableSingleCaliper(const QString& toolId, double centerX, double centerY, double width, double height, double searchDirectionAngleDeg);
    Q_INVOKABLE QVariantMap editableCaliperGeometry(const QString& toolId) const;

signals:
    void zoomChanged();
    void minZoomChanged();
    void maxZoomChanged();
    void showPixelInfoChanged();
    void showCrosshairChanged();
    void interactionModeChanged();
    void selectedRoiIdChanged();
    void autoFitOnNewImageChanged();
    void keepViewTransformOnNewImageChanged();
    void modelRoiChanged();
    void modelRoiVisibleChanged();
    void modelRoiEditableChanged();

    void mouseImagePositionChanged(double x, double y);
    void imageClicked(double x, double y);
    void imageDoubleClicked(double x, double y);
    void imageRightClicked(double x, double y);

    void roiCreated(const QString& id);
    void roiChanged(const QString& id);
    void roiSelected(const QString& id);
    void roiDeleted(const QString& id);

protected:
    QSGNode* updatePaintNode(QSGNode* oldNode, UpdatePaintNodeData*) override;

    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;
    void hoverMoveEvent(QHoverEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void geometryChange(const QRectF& newGeometry, const QRectF& oldGeometry) override;

private:
    struct RoiDragSnapshot
    {
        QString id;
        QPointF pressImagePoint;
        RoiType type = RoiType::Rect;
        QRectF rect;
        QPointF center;
        double width = 0.0;
        double height = 0.0;
        double angleDeg = 0.0;
        double radius = 0.0;
        QLineF line;
        int handle = -1;
        bool valid = false;
    };

    enum class EditableCaliperKind {
        Line,
        Circle,
        Ellipse,
        Single
    };

    struct EditableCaliperArray
    {
        QString toolId;
        EditableCaliperKind kind = EditableCaliperKind::Line;
        QPointF p1;
        QPointF p2;
        QPointF center;
        double radius = 0.0;
        double radiusX = 0.0;
        double radiusY = 0.0;
        double angleDeg = 0.0;
        double caliperWidth = 20.0;
        double caliperHeight = 80.0;
        int count = 8;
    };

    struct EditableCaliperDrag
    {
        bool active = false;
        QString toolId;
        int handle = -1;
        EditableCaliperArray original;
        QPointF pressImagePoint;
    };

    struct ModelRoiDrag
    {
        bool active = false;
        int handle = -1;
        QRectF originalRoi;
        QPointF pressImagePoint;
    };

    double boundedZoom(double zoom) const;
    QRectF imageViewRect() const;
    void updateMapperViewSize();
    void emitMouseImagePosition(const QPointF& viewPoint);
    double imageHitTolerance(double viewTolerance = 6.0) const;
    void selectRoi(const QString& id);
    bool beginRoiDrag(const QString& id, const QPointF& pressImagePoint);
    bool updateDraggedRoi(const QPointF& currentImagePoint);
    bool beginEditableCaliperDrag(const QPointF& imagePoint);
    bool updateEditableCaliperDrag(const QPointF& imagePoint);
    int modelRoiHandleAt(const QPointF& imagePoint) const;
    bool beginModelRoiDrag(const QPointF& imagePoint);
    bool updateModelRoiDrag(const QPointF& imagePoint);
    QRectF boundedModelRoi(const QRectF& roi) const;
    QRectF defaultModelRoi() const;
    void updateModelRoiCursor(const QPointF& imagePoint);
    void rebuildEditableCalipers(const QString& toolId, bool requestUpdate = true);
    void removeEditableCalipers(const QString& toolId);
    QVector<QPointF> pointsFromVariantList(const QVariantList& points) const;
    void addResultGraphic(const GraphicObject& graphic, bool requestUpdate = true);
    void addToolGraphic(const GraphicObject& graphic, bool requestUpdate = true);
    void applyFrame(FrameData frame);
    static FrameData frameFromImage(const QImage& image);
    static FrameData frameFromRaw(const uchar* data, int width, int height, int stride, PixelFormat pixelFormat);
    void markOverlayDirty();

    QImage m_image;
    quint64 m_frameSequence = 0;
    bool m_textureDirty = true;
    bool m_overlayDirty = true;
    bool m_autoFitOnNewImage = true;
    bool m_keepViewTransformOnNewImage = false;
    CoordinateMapper m_mapper;
    RoiManager m_rois;
    double m_minZoom = 0.05;
    double m_maxZoom = 100.0;
    bool m_showPixelInfo = true;
    bool m_showCrosshair = false;
    int m_interactionMode = PanMode;
    bool m_panning = false;
    bool m_draggingRoi = false;
    QPointF m_lastPanPoint;
    RoiDragSnapshot m_roiDragSnapshot;
    GraphicManager m_graphics;
    QRectF m_modelRoi;
    bool m_modelRoiVisible = false;
    bool m_modelRoiEditable = false;
    ModelRoiDrag m_modelRoiDrag;
    QList<EditableCaliperArray> m_editableCalipers;
    EditableCaliperDrag m_editableCaliperDrag;
};

} // namespace VisionDisplay
