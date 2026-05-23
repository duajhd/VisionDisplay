#include "IntegratedDemoController.h"

#include "ToolResultDisplayAdapter.h"
#include "VisionDisplay/VisionDisplayItem.h"
#include "VisionTools/Backend/DependencySelfTest.h"
#include "VisionTools/CaliperTool.h"
#include "VisionTools/FindCircleTool.h"
#include "VisionTools/FindEllipseTool.h"
#include "VisionTools/FindLineTool.h"
#include "VisionTools/ImageView.h"
#include "VisionTools/Matching/GrayMatcher.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QImageReader>
#include <QDebug>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLineF>
#include <QPainter>
#include <QPointF>
#include <QUrl>
#include <QVariantMap>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <QStringList>

namespace {

constexpr double pi = 3.14159265358979323846;
constexpr double lineAngleDeg = 8.0;
constexpr double lineCenterX = 260.0;
constexpr double lineCenterY = 300.0;
constexpr double circleCenterX = 560.0;
constexpr double circleCenterY = 320.0;
constexpr double circleRadius = 96.0;
constexpr double regressionCircleCenterX = 400.0;
constexpr double regressionCircleCenterY = 300.0;
constexpr double regressionCircleRadius = 120.0;
constexpr double ellipseCenterX = 400.0;
constexpr double ellipseCenterY = 300.0;
constexpr double ellipseRadiusA = 180.0;
constexpr double ellipseRadiusB = 90.0;
constexpr double ellipseAngleDeg = 25.0;
constexpr char findLineRegionToolId[] = "FindLineRegion_1";
constexpr char findCircleCalipersToolId[] = "FindCircleCalipers_1";
constexpr char singleCaliperToolId[] = "SingleCaliperDebug_1";
constexpr char grayMatchToolId[] = "GrayMatch_1";

VisionTools::RobustLossType robustLossFromName(const QString& lossType)
{
    if (lossType.compare(QStringLiteral("None"), Qt::CaseInsensitive) == 0) {
        return VisionTools::RobustLossType::None;
    }
    if (lossType.compare(QStringLiteral("Tukey"), Qt::CaseInsensitive) == 0) {
        return VisionTools::RobustLossType::Tukey;
    }
    return VisionTools::RobustLossType::Huber;
}

QString robustLossName(VisionTools::RobustLossType lossType)
{
    switch (lossType) {
    case VisionTools::RobustLossType::None:
        return QStringLiteral("None");
    case VisionTools::RobustLossType::Huber:
        return QStringLiteral("Huber");
    case VisionTools::RobustLossType::Tukey:
        return QStringLiteral("Tukey");
    }
    return QStringLiteral("Huber");
}

VisionTools::EdgePolarity edgePolarityFromName(const QString& name)
{
    if (name.compare(QStringLiteral("DarkToLight"), Qt::CaseInsensitive) == 0) {
        return VisionTools::EdgePolarity::DarkToLight;
    }
    if (name.compare(QStringLiteral("LightToDark"), Qt::CaseInsensitive) == 0) {
        return VisionTools::EdgePolarity::LightToDark;
    }
    return VisionTools::EdgePolarity::Any;
}

VisionTools::EdgePolarity edgePolarityFromIndex(int index)
{
    switch (index) {
    case 0:
        return VisionTools::EdgePolarity::DarkToLight;
    case 1:
        return VisionTools::EdgePolarity::LightToDark;
    case 2:
        return VisionTools::EdgePolarity::Any;
    default:
        return VisionTools::EdgePolarity::Any;
    }
}

QString edgePolarityName(VisionTools::EdgePolarity polarity)
{
    switch (polarity) {
    case VisionTools::EdgePolarity::DarkToLight:
        return QStringLiteral("DarkToLight");
    case VisionTools::EdgePolarity::LightToDark:
        return QStringLiteral("LightToDark");
    case VisionTools::EdgePolarity::Any:
        return QStringLiteral("Any");
    }
    return QStringLiteral("Any");
}

double findLineSearchDirectionAngleDeg(int index)
{
    switch (index) {
    case 0:
        return 90.0;
    case 1:
        return -90.0;
    case 2:
        return 0.0;
    case 3:
        return 180.0;
    default:
        return 90.0;
    }
}

QString findLineSearchDirectionName(int index)
{
    switch (index) {
    case 0:
        return QStringLiteral("TopToBottom");
    case 1:
        return QStringLiteral("BottomToTop");
    case 2:
        return QStringLiteral("LeftToRight");
    case 3:
        return QStringLiteral("RightToLeft");
    default:
        return QStringLiteral("TopToBottom");
    }
}

VisionTools::EdgeSelection edgeSelectionFromName(const QString& name)
{
    if (name.compare(QStringLiteral("First"), Qt::CaseInsensitive) == 0) {
        return VisionTools::EdgeSelection::First;
    }
    if (name.compare(QStringLiteral("Last"), Qt::CaseInsensitive) == 0) {
        return VisionTools::EdgeSelection::Last;
    }
    if (name.compare(QStringLiteral("NearestToCenter"), Qt::CaseInsensitive) == 0) {
        return VisionTools::EdgeSelection::NearestToCenter;
    }
    if (name.compare(QStringLiteral("NearestToExpected"), Qt::CaseInsensitive) == 0) {
        return VisionTools::EdgeSelection::NearestToExpected;
    }
    if (name.compare(QStringLiteral("All"), Qt::CaseInsensitive) == 0) {
        return VisionTools::EdgeSelection::All;
    }
    return VisionTools::EdgeSelection::Strongest;
}

VisionTools::EdgeSelection edgeSelectionFromIndex(int index)
{
    switch (index) {
    case 0:
        return VisionTools::EdgeSelection::Strongest;
    case 1:
        return VisionTools::EdgeSelection::First;
    case 2:
        return VisionTools::EdgeSelection::Last;
    case 3:
        return VisionTools::EdgeSelection::NearestToCenter;
    case 4:
        return VisionTools::EdgeSelection::NearestToExpected;
    case 5:
        return VisionTools::EdgeSelection::All;
    default:
        return VisionTools::EdgeSelection::Strongest;
    }
}

QString edgeSelectionName(VisionTools::EdgeSelection selection)
{
    switch (selection) {
    case VisionTools::EdgeSelection::First:
        return QStringLiteral("First");
    case VisionTools::EdgeSelection::Last:
        return QStringLiteral("Last");
    case VisionTools::EdgeSelection::Strongest:
        return QStringLiteral("Strongest");
    case VisionTools::EdgeSelection::NearestToCenter:
        return QStringLiteral("NearestToCenter");
    case VisionTools::EdgeSelection::NearestToExpected:
        return QStringLiteral("NearestToExpected");
    case VisionTools::EdgeSelection::All:
        return QStringLiteral("All");
    }
    return QStringLiteral("Strongest");
}

double polarityResponse(double gradient, VisionTools::EdgePolarity polarity)
{
    switch (polarity) {
    case VisionTools::EdgePolarity::DarkToLight:
        return gradient;
    case VisionTools::EdgePolarity::LightToDark:
        return -gradient;
    case VisionTools::EdgePolarity::Any:
        return std::abs(gradient);
    }
    return 0.0;
}

QString formatDoubleArray(const std::vector<double>& values, int precision = 3)
{
    QStringList items;
    items.reserve(static_cast<int>(values.size()));
    for (double value : values) {
        items << QString::number(value, 'f', precision);
    }
    return QStringLiteral("[%1]").arg(items.join(QStringLiteral(", ")));
}

QString formatFindCircleDiagnostics(const VisionTools::FindCircleResult& result)
{
    QStringList lines;
    lines << QStringLiteral("FindCircle Diagnostics");
    lines << QStringLiteral("caliperCount=%1").arg(result.diagnostics.caliperCount);
    lines << QStringLiteral("selectedEdgeCount=%1").arg(result.diagnostics.selectedEdgeCount);
    lines << QStringLiteral("acceptedFitPointsCount=%1").arg(result.diagnostics.fitInputPointsCount);
    lines << QStringLiteral("rejectedByExpectedWindowCount=%1").arg(result.diagnostics.rejectedByExpectedWindowCount);
    lines << QStringLiteral("rejectedPointCount=%1").arg(static_cast<int>(result.rejectedEdgePoints.size()));
    lines << QStringLiteral("inlierCount=%1").arg(result.diagnostics.inlierCount);
    lines << QStringLiteral("outlierCount=%1").arg(result.diagnostics.outlierCount);
    lines << QStringLiteral("centerX=%1").arg(result.fitResult.circle.centerX, 0, 'f', 3);
    lines << QStringLiteral("centerY=%1").arg(result.fitResult.circle.centerY, 0, 'f', 3);
    lines << QStringLiteral("radius=%1").arg(result.fitResult.circle.radius, 0, 'f', 3);
    lines << QStringLiteral("rmsError=%1").arg(result.fitResult.rmsError, 0, 'f', 6);
    lines << QStringLiteral("score=%1").arg(result.fitResult.score, 0, 'f', 6);
    lines << QStringLiteral("message=%1").arg(result.message.isEmpty() && result.ok ? QStringLiteral("OK") : result.message);
    return lines.join(QLatin1Char('\n'));
}

QString formatFindLineDiagnostics(const VisionTools::FindLineResult& result)
{
    QStringList lines;
    lines << QStringLiteral("FindLine Diagnostics");
    lines << QStringLiteral("caliperCount=%1").arg(static_cast<int>(result.calipers.size()));
    lines << QStringLiteral("lineAngleDeg=%1").arg(result.searchRegion.angleDeg, 0, 'f', 3);
    lines << QStringLiteral("searchDirectionAngleDeg=%1").arg(result.searchRegion.searchDirectionAngleDeg, 0, 'f', 3);
    lines << QStringLiteral("selectedEdgeCount=%1").arg(static_cast<int>(result.edgePoints.size()));
    lines << QStringLiteral("inputPointCount=%1").arg(static_cast<int>(result.inputPoints.size()));
    lines << QStringLiteral("inlierCount=%1").arg(static_cast<int>(result.inlierPoints.size()));
    lines << QStringLiteral("outlierCount=%1").arg(static_cast<int>(result.outlierPoints.size()));
    lines << QStringLiteral("x1=%1").arg(result.fitResult.line.x1, 0, 'f', 3);
    lines << QStringLiteral("y1=%1").arg(result.fitResult.line.y1, 0, 'f', 3);
    lines << QStringLiteral("x2=%1").arg(result.fitResult.line.x2, 0, 'f', 3);
    lines << QStringLiteral("y2=%1").arg(result.fitResult.line.y2, 0, 'f', 3);
    lines << QStringLiteral("rmsError=%1").arg(result.rmsError, 0, 'f', 6);
    lines << QStringLiteral("maxError=%1").arg(result.maxError, 0, 'f', 6);
    lines << QStringLiteral("score=%1").arg(result.score, 0, 'f', 6);
    lines << QStringLiteral("message=%1").arg(result.message.isEmpty() && result.ok ? QStringLiteral("OK") : result.message);
    return lines.join(QLatin1Char('\n'));
}

bool isFiniteEdgePoint(const VisionTools::EdgePoint& edge);

int selectedEdgeIndex(const std::vector<VisionTools::EdgePoint>& candidates,
                      const std::vector<VisionTools::EdgePoint>& selectedEdges)
{
    if (candidates.empty() || selectedEdges.empty()) {
        return -1;
    }

    const VisionTools::EdgePoint selected = selectedEdges.front();
    if (!isFiniteEdgePoint(selected)) {
        return -1;
    }
    int bestIndex = -1;
    double bestDistance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (!isFiniteEdgePoint(candidates[i])) {
            continue;
        }
        const double dx = candidates[i].x - selected.x;
        const double dy = candidates[i].y - selected.y;
        const double distance = dx * dx + dy * dy;
        if (distance < bestDistance) {
            bestDistance = distance;
            bestIndex = static_cast<int>(i);
        }
    }
    return bestIndex;
}

QVariantMap edgePointMap(const VisionTools::EdgePoint& edge,
                         int index,
                         bool selected,
                         const QString& polarity,
                         bool withinExpected)
{
    QVariantMap point;
    point.insert(QStringLiteral("id"), QStringLiteral("candidate_%1").arg(index));
    point.insert(QStringLiteral("index"), index);
    point.insert(QStringLiteral("x"), edge.x);
    point.insert(QStringLiteral("y"), edge.y);
    point.insert(QStringLiteral("nx"), edge.nx);
    point.insert(QStringLiteral("ny"), edge.ny);
    point.insert(QStringLiteral("response"), edge.response);
    point.insert(QStringLiteral("position1D"), edge.position1D);
    point.insert(QStringLiteral("subIndex"), edge.subIndex);
    point.insert(QStringLiteral("valid"), edge.valid);
    point.insert(QStringLiteral("selected"), selected);
    point.insert(QStringLiteral("polarity"), polarity);
    point.insert(QStringLiteral("withinExpected"), withinExpected);
    return point;
}

QJsonObject edgePointJson(const VisionTools::EdgePoint& edge,
                          int index,
                          bool selected,
                          const QString& polarity,
                          bool withinExpected)
{
    QJsonObject point;
    point.insert(QStringLiteral("id"), QStringLiteral("candidate_%1").arg(index));
    point.insert(QStringLiteral("index"), index);
    point.insert(QStringLiteral("x"), edge.x);
    point.insert(QStringLiteral("y"), edge.y);
    point.insert(QStringLiteral("nx"), edge.nx);
    point.insert(QStringLiteral("ny"), edge.ny);
    point.insert(QStringLiteral("response"), edge.response);
    point.insert(QStringLiteral("position1D"), edge.position1D);
    point.insert(QStringLiteral("subIndex"), edge.subIndex);
    point.insert(QStringLiteral("valid"), edge.valid);
    point.insert(QStringLiteral("selected"), selected);
    point.insert(QStringLiteral("polarity"), polarity);
    point.insert(QStringLiteral("withinExpected"), withinExpected);
    return point;
}

QVariantMap profilePeakMap(int index,
                           double subIndex,
                           double position1D,
                           double response,
                           double gradient,
                           bool aboveMinResponse)
{
    QVariantMap peak;
    peak.insert(QStringLiteral("index"), index);
    peak.insert(QStringLiteral("subIndex"), subIndex);
    peak.insert(QStringLiteral("position1D"), position1D);
    peak.insert(QStringLiteral("response"), response);
    peak.insert(QStringLiteral("gradient"), gradient);
    peak.insert(QStringLiteral("aboveMinResponse"), aboveMinResponse);
    return peak;
}

QJsonObject profilePeakJson(int index,
                            double subIndex,
                            double position1D,
                            double response,
                            double gradient,
                            bool aboveMinResponse)
{
    QJsonObject peak;
    peak.insert(QStringLiteral("index"), index);
    peak.insert(QStringLiteral("subIndex"), subIndex);
    peak.insert(QStringLiteral("position1D"), position1D);
    peak.insert(QStringLiteral("response"), response);
    peak.insert(QStringLiteral("gradient"), gradient);
    peak.insert(QStringLiteral("aboveMinResponse"), aboveMinResponse);
    return peak;
}

bool isFiniteEdgePoint(const VisionTools::EdgePoint& edge)
{
    return std::isfinite(edge.x)
        && std::isfinite(edge.y)
        && std::isfinite(edge.nx)
        && std::isfinite(edge.ny)
        && std::isfinite(edge.response)
        && std::isfinite(edge.position1D)
        && std::isfinite(edge.subIndex);
}

QString displayNameFromPath(const QString& path)
{
    const int slashIndex = std::max(path.lastIndexOf(QLatin1Char('/')),
                                    path.lastIndexOf(QLatin1Char('\\')));
    if (slashIndex >= 0 && slashIndex + 1 < path.size()) {
        return path.mid(slashIndex + 1);
    }
    return path;
}

} // namespace

IntegratedDemoController::IntegratedDemoController(QObject* parent)
    : QObject(parent)
    , m_status(QStringLiteral("Ready"))
{
}

QString IntegratedDemoController::status() const
{
    return m_status;
}

void IntegratedDemoController::bindDisplay(QObject* displayObject)
{
    m_display = qobject_cast<VisionDisplay::VisionDisplayItem*>(displayObject);
    setStatus(QStringLiteral("Ready. Load an image or reset a synthetic image."));
}

void IntegratedDemoController::resetImage()
{
    m_image = createSyntheticImage();
    m_imageKind = DemoImageKind::CircleSynthetic;
    if (m_display) {
        m_display->setKeepViewTransformOnNewImage(false);
        m_display->setAutoFitOnNewImage(true);
        m_display->setImage(m_image);
        m_display->fitToWindow();
        m_display->clearAllToolGraphics();
    }
    setStatus(QStringLiteral("Synthetic circle image generated. Expected C=(%1,%2), R=%3")
                  .arg(circleCenterX, 0, 'f', 1)
                  .arg(circleCenterY, 0, 'f', 1)
                  .arg(circleRadius, 0, 'f', 1));
}

void IntegratedDemoController::resetEllipseImage()
{
    if (!m_display) {
        setStatus(QStringLiteral("Display not ready"));
        return;
    }

    m_image = createEllipseImage();
    m_imageKind = DemoImageKind::EllipseSynthetic;
    m_display->setKeepViewTransformOnNewImage(false);
    m_display->setAutoFitOnNewImage(true);
    m_display->setImage(m_image);
    m_display->fitToWindow();
    m_display->clearAllToolGraphics();
    setStatus(QStringLiteral("Synthetic ellipse image generated"));
}

void IntegratedDemoController::generateCircleTestImage()
{
    if (!m_display) {
        setStatus(QStringLiteral("Display not ready"));
        return;
    }

    m_image = createCircleRegressionImage();
    m_imageKind = DemoImageKind::CircleSynthetic;
    m_display->setKeepViewTransformOnNewImage(false);
    m_display->setAutoFitOnNewImage(true);
    m_display->setImage(m_image);
    m_display->fitToWindow();
    m_display->clearAllToolGraphics();
    m_circleDiagnostics.clear();
    setStatus(QStringLiteral("Circle regression image generated. Expected C=(400.0,300.0), R=120.0"));
}

bool IntegratedDemoController::loadImage(const QString& pathOrUrl)
{
    const QString path = localFilePath(pathOrUrl);
    QImageReader reader(path);
    reader.setAutoTransform(true);
    QImage image = reader.read();
    if (image.isNull()) {
        setStatus(QStringLiteral("Load image failed: %1").arg(reader.errorString()));
        return false;
    }

    if (image.format() != QImage::Format_Grayscale8) {
        image = image.convertToFormat(QImage::Format_Grayscale8);
    }

    m_image = image;
    m_imageKind = DemoImageKind::External;
    if (m_display) {
        m_display->setKeepViewTransformOnNewImage(false);
        m_display->setAutoFitOnNewImage(true);
        m_display->setImage(m_image);
        m_display->fitToWindow();
        m_display->clearAllToolGraphics();
    }
    setStatus(QStringLiteral("Loaded %1 (%2x%3)")
                  .arg(displayNameFromPath(path))
                  .arg(m_image.width())
                  .arg(m_image.height()));
    return true;
}

void IntegratedDemoController::createFindCircleCalipers()
{
    if (!m_display || m_image.isNull()) {
        setStatus(QStringLiteral("Load an image first"));
        return;
    }

    if (m_imageKind == DemoImageKind::CircleSynthetic) {
        createSyntheticCircleCalipers();
        setStatus(QStringLiteral("Circle calipers aligned to synthetic circle. Drag handles for manual regression checks."));
        return;
    }

    m_display->createEditableCircleCalipers(QString::fromLatin1(findCircleCalipersToolId),
                                            m_image.width() * 0.5,
                                            m_image.height() * 0.5,
                                            std::min(m_image.width(), m_image.height()) * 0.25,
                                            48.0,
                                            220.0,
                                            48);
    setStatus(QStringLiteral("Circle calipers created. Drag center to move; drag red handle to resize."));
}

void IntegratedDemoController::runCaliper()
{
    if (!m_display || m_image.isNull()) {
        setStatus(QStringLiteral("Display or image not ready"));
        return;
    }

    VisionTools::CaliperRegion region;
    region.centerX = lineCenterX;
    region.centerY = lineCenterY;
    region.length = 130.0;
    region.width = 42.0;
    region.angleDeg = lineAngleDeg + 90.0;

    VisionTools::CaliperParams params;
    params.sampleCount = 121;
    params.projectionCount = 9;
    params.smoothingSigma = 1.0;
    params.minResponse = 5.0;
    params.polarity = VisionTools::EdgePolarity::DarkToLight;
    params.selection = VisionTools::EdgeSelection::Strongest;
    params.enableSubpixel = true;

    VisionTools::CaliperTool tool;
    tool.setParams(params);
    const VisionTools::CaliperResult result = tool.run(VisionTools::ImageView(m_image), region);

    ToolResultDisplayAdapter::showCaliper(m_display, QStringLiteral("Caliper_1"), region, result);
    setStatus(QStringLiteral("Caliper_1 %1 candidates=%2")
                  .arg(result.ok ? QStringLiteral("OK") : QStringLiteral("NG"))
                  .arg(static_cast<int>(result.candidates.size())));
}

void IntegratedDemoController::runFindLine()
{
    runFindLineWithParams(0, 5.0, 0, 15, 20.0, 120.0);
}

void IntegratedDemoController::runFindLineWithParams(int polarity,
                                                     double minResponse,
                                                     int searchDirection,
                                                     int caliperCount,
                                                     double projectionWidth,
                                                     double searchLength)
{
    if (!m_display || m_image.isNull()) {
        setStatus(QStringLiteral("Display or image not ready"));
        return;
    }

    VisionTools::LineSearchRegion region;
    QVariantMap editableRegion = m_display->editableCaliperGeometry(QString::fromLatin1(findLineRegionToolId));
    if (!editableRegion.value(QStringLiteral("valid")).toBool()
        || editableRegion.value(QStringLiteral("type")).toString() != QStringLiteral("line")) {
        m_display->createEditableLineCalipers(QString::fromLatin1(findLineRegionToolId),
                                              lineCenterX - 180.0,
                                              lineCenterY - std::tan(lineAngleDeg * pi / 180.0) * 180.0,
                                              lineCenterX + 180.0,
                                              lineCenterY + std::tan(lineAngleDeg * pi / 180.0) * 180.0,
                                              20.0,
                                              120.0,
                                              15);
        editableRegion = m_display->editableCaliperGeometry(QString::fromLatin1(findLineRegionToolId));
    }

    const QPointF p1(editableRegion.value(QStringLiteral("x1")).toDouble(),
                     editableRegion.value(QStringLiteral("y1")).toDouble());
    const QPointF p2(editableRegion.value(QStringLiteral("x2")).toDouble(),
                     editableRegion.value(QStringLiteral("y2")).toDouble());
    const QLineF editableLine(p1, p2);
    if (editableLine.length() < 1.0) {
        setStatus(QStringLiteral("FindLine region is too short"));
        return;
    }

    const QPointF center = (p1 + p2) * 0.5;
    region.centerX = center.x();
    region.centerY = center.y();
    region.length = editableLine.length();
    region.searchLength = std::max(1.0, searchLength);
    region.angleDeg = std::atan2(p2.y() - p1.y(), p2.x() - p1.x()) * 180.0 / pi;
    region.searchDirectionAngleDeg = findLineSearchDirectionAngleDeg(searchDirection);
    region.caliperCount = std::max(2, caliperCount);
    region.caliperWidth = std::max(1.0, projectionWidth);

    VisionTools::FindLineParams params;
    params.caliperParams.sampleCount = 101;
    params.caliperParams.projectionCount = 7;
    params.caliperParams.smoothingSigma = 1.0;
    params.caliperParams.minResponse = std::max(0.0, minResponse);
    params.caliperParams.polarity = edgePolarityFromIndex(polarity);
    params.caliperParams.selection = VisionTools::EdgeSelection::NearestToExpected;
    params.caliperParams.expectedPosition1D = 0.0;
    params.caliperParams.maxPositionDeviation = region.searchLength * 0.5;
    params.caliperParams.allowFallbackSelection = false;
    params.caliperParams.enableSubpixel = true;
    params.fitParams.maxResidual = 2.5;
    params.fitParams.minInlierCount = 6;
    params.minEdgeResponse = params.caliperParams.minResponse;

    VisionTools::FindLineTool tool;
    tool.setParams(params);
    const VisionTools::FindLineResult result = tool.run(VisionTools::ImageView(m_image), region);
    m_findLineDiagnostics = formatFindLineDiagnostics(result);
    m_findLineDiagnostics += QStringLiteral("\npolarity=%1\nsearchDirection=%2")
                                 .arg(edgePolarityName(params.caliperParams.polarity),
                                      findLineSearchDirectionName(searchDirection));
    qDebug().noquote() << m_findLineDiagnostics;

    ToolResultDisplayAdapter::showFindLine(m_display, QStringLiteral("FindLine_1"), region, result);
    setStatus(QStringLiteral("FindLine_1 %1 points=%2 rms=%3")
                  .arg(result.ok ? QStringLiteral("OK") : QStringLiteral("NG"))
                  .arg(static_cast<int>(result.edgePoints.size()))
                  .arg(result.fitResult.rmsError, 0, 'f', 3));
}

QString IntegratedDemoController::runFindLineStabilityTest(int polarity,
                                                           double minResponse,
                                                           int searchDirection,
                                                           int caliperCount,
                                                           double projectionWidth,
                                                           double searchLength)
{
    if (!m_display || m_image.isNull()) {
        m_findLineDiagnostics = QStringLiteral("Display or image not ready");
        setStatus(m_findLineDiagnostics);
        return m_findLineDiagnostics;
    }

    QVariantMap editableRegion = m_display->editableCaliperGeometry(QString::fromLatin1(findLineRegionToolId));
    if (!editableRegion.value(QStringLiteral("valid")).toBool()
        || editableRegion.value(QStringLiteral("type")).toString() != QStringLiteral("line")) {
        m_display->createEditableLineCalipers(QString::fromLatin1(findLineRegionToolId),
                                              lineCenterX - 180.0,
                                              lineCenterY - std::tan(lineAngleDeg * pi / 180.0) * 180.0,
                                              lineCenterX + 180.0,
                                              lineCenterY + std::tan(lineAngleDeg * pi / 180.0) * 180.0,
                                              20.0,
                                              120.0,
                                              15);
        editableRegion = m_display->editableCaliperGeometry(QString::fromLatin1(findLineRegionToolId));
    }

    const QPointF p1(editableRegion.value(QStringLiteral("x1")).toDouble(),
                     editableRegion.value(QStringLiteral("y1")).toDouble());
    const QPointF p2(editableRegion.value(QStringLiteral("x2")).toDouble(),
                     editableRegion.value(QStringLiteral("y2")).toDouble());
    const QLineF editableLine(p1, p2);
    if (editableLine.length() < 1.0) {
        m_findLineDiagnostics = QStringLiteral("FindLine region is too short");
        setStatus(m_findLineDiagnostics);
        return m_findLineDiagnostics;
    }

    VisionTools::LineSearchRegion region;
    const QPointF center = (p1 + p2) * 0.5;
    region.centerX = center.x();
    region.centerY = center.y();
    region.length = editableLine.length();
    region.searchLength = std::max(1.0, searchLength);
    region.angleDeg = std::atan2(p2.y() - p1.y(), p2.x() - p1.x()) * 180.0 / pi;
    region.searchDirectionAngleDeg = findLineSearchDirectionAngleDeg(searchDirection);
    region.caliperCount = std::max(2, caliperCount);
    region.caliperWidth = std::max(1.0, projectionWidth);

    VisionTools::FindLineParams params;
    params.caliperParams.sampleCount = 101;
    params.caliperParams.projectionCount = 7;
    params.caliperParams.smoothingSigma = 1.0;
    params.caliperParams.minResponse = std::max(0.0, minResponse);
    params.caliperParams.polarity = edgePolarityFromIndex(polarity);
    params.caliperParams.selection = VisionTools::EdgeSelection::NearestToExpected;
    params.caliperParams.expectedPosition1D = 0.0;
    params.caliperParams.maxPositionDeviation = region.searchLength * 0.5;
    params.caliperParams.allowFallbackSelection = false;
    params.caliperParams.enableSubpixel = true;
    params.fitParams.maxResidual = 2.5;
    params.fitParams.minInlierCount = 6;
    params.minEdgeResponse = params.caliperParams.minResponse;

    VisionTools::FindLineTool tool;
    tool.setParams(params);

    QStringList lines;
    int okCount = 0;
    VisionTools::FindLineResult lastResult;
    lines << QStringLiteral("FindLine Stability Test");
    for (int i = 0; i < 20; ++i) {
        lastResult = tool.run(VisionTools::ImageView(m_image), region);
        if (lastResult.ok) {
            ++okCount;
        }
        QString line = QStringLiteral("#%1 ok=%2 points=%3 rms=%4 score=%5")
                           .arg(i + 1, 2, 10, QLatin1Char('0'))
                           .arg(lastResult.ok ? QStringLiteral("true") : QStringLiteral("false"))
                           .arg(static_cast<int>(lastResult.edgePoints.size()))
                           .arg(lastResult.rmsError, 0, 'f', 6)
                           .arg(lastResult.score, 0, 'f', 6);
        if (!lastResult.ok) {
            line += QStringLiteral(" message=%1").arg(lastResult.message);
        }
        lines << line;
    }
    lines << QStringLiteral("summary okCount=%1/20 finalMessage=%2")
                 .arg(okCount)
                 .arg(lastResult.message.isEmpty() && lastResult.ok ? QStringLiteral("OK") : lastResult.message);
    lines << QStringLiteral("polarity=%1").arg(edgePolarityName(params.caliperParams.polarity));
    lines << QStringLiteral("searchDirection=%1").arg(findLineSearchDirectionName(searchDirection));
    lines << formatFindLineDiagnostics(lastResult);

    m_findLineDiagnostics = lines.join(QLatin1Char('\n'));
    qDebug().noquote() << m_findLineDiagnostics;
    ToolResultDisplayAdapter::showFindLine(m_display, QStringLiteral("FindLine_1"), region, lastResult);
    setStatus(QStringLiteral("FindLine stability %1/20 OK").arg(okCount));
    return m_findLineDiagnostics;
}

void IntegratedDemoController::runFindCircle()
{
    runFindCircleWithParams(0, 5.0, 1, 48, 48.0, 120.0);
}

void IntegratedDemoController::runFindCircleWithParams(int polarity,
                                                       double minResponse,
                                                       int searchDirection,
                                                       int caliperCount,
                                                       double projectionWidth,
                                                       double searchLength)
{
    if (!m_display || m_image.isNull()) {
        setStatus(QStringLiteral("Display or image not ready"));
        return;
    }

    if (m_imageKind == DemoImageKind::EllipseSynthetic) {
        setStatus(QStringLiteral("Current image is the ellipse regression image. Click Reset Circle Image before Run FindCircle."));
        return;
    }

    QVariantMap calipers = m_display->editableCaliperGeometry(QString::fromLatin1(findCircleCalipersToolId));
    if (!calipers.value(QStringLiteral("valid")).toBool() || calipers.value(QStringLiteral("type")).toString() != QStringLiteral("circle")) {
        createFindCircleCalipers();
        calipers = m_display->editableCaliperGeometry(QString::fromLatin1(findCircleCalipersToolId));
    }

    VisionTools::CircleSearchRegion region;
    region.centerX = calipers.value(QStringLiteral("centerX")).toDouble();
    region.centerY = calipers.value(QStringLiteral("centerY")).toDouble();
    region.radius = calipers.value(QStringLiteral("radius")).toDouble();
    region.searchLength = std::max(1.0, searchLength);
    region.caliperWidth = std::max(1.0, projectionWidth);
    region.startAngleDeg = 0.0;
    region.spanAngleDeg = 360.0;
    region.caliperCount = std::max(3, caliperCount);
    region.searchDirection = searchDirection == 0
        ? VisionTools::CircleSearchDirection::Inward
        : VisionTools::CircleSearchDirection::Outward;

    VisionTools::FindCircleParams params;
    params.caliperParams.sampleCount = 161;
    params.caliperParams.projectionCount = 9;
    params.caliperParams.smoothingSigma = 1.0;
    params.caliperParams.minResponse = std::max(0.0, minResponse);
    params.caliperParams.polarity = edgePolarityFromIndex(polarity);
    params.caliperParams.selection = VisionTools::EdgeSelection::Strongest;
    params.caliperParams.enableSubpixel = true;
    params.fitParams.maxResidual = 3.0;
    params.fitParams.minInlierCount = 12;
    params.fitParams.maxIterations = 20;
    params.fitParams.damping = 1.0e-3;
    params.fitParams.ransacResidual = 4.0;
    params.fitParams.huberDelta = 2.0;
    params.minEdgeResponse = params.caliperParams.minResponse;
    params.circleExpectedPositionTolerance = region.searchLength * 0.5;

    VisionTools::FindCircleTool tool;
    tool.setParams(params);
    const VisionTools::FindCircleResult result = tool.run(VisionTools::ImageView(m_image), region);
    m_findCircleDiagnostics = formatFindCircleDiagnostics(result);
    qDebug().noquote() << m_findCircleDiagnostics;

    m_display->clearToolGraphics(QStringLiteral("FindEllipse_1"));
    ToolResultDisplayAdapter::showFindCircle(m_display, QStringLiteral("FindCircle_1"), region, result);
    QString status = QStringLiteral("FindCircle_1 %1 points=%2 C=(%3,%4) R=%5 RMS=%6")
                  .arg(result.ok ? QStringLiteral("OK") : QStringLiteral("NG"))
                  .arg(result.diagnostics.fitInputPointsCount)
                  .arg(result.fitResult.circle.centerX, 0, 'f', 2)
                  .arg(result.fitResult.circle.centerY, 0, 'f', 2)
                  .arg(result.fitResult.circle.radius, 0, 'f', 2)
                  .arg(result.fitResult.rmsError, 0, 'f', 3);
    if (!result.ok && !result.message.isEmpty()) {
        status += QStringLiteral(" message=%1").arg(result.message);
    }
    if (m_imageKind == DemoImageKind::CircleSynthetic && result.fitResult.ok) {
        status += QStringLiteral(" dC=(%1,%2) dR=%3")
                      .arg(result.fitResult.circle.centerX - circleCenterX, 0, 'f', 2)
                      .arg(result.fitResult.circle.centerY - circleCenterY, 0, 'f', 2)
                      .arg(result.fitResult.circle.radius - circleRadius, 0, 'f', 2);
    }
    setStatus(status);
}

QString IntegratedDemoController::runFindCircleStabilityTest(int polarity,
                                                             double minResponse,
                                                             int searchDirection,
                                                             int caliperCount,
                                                             double projectionWidth,
                                                             double searchLength)
{
    if (!m_display || m_image.isNull()) {
        m_findCircleDiagnostics = QStringLiteral("Display or image not ready");
        setStatus(m_findCircleDiagnostics);
        return m_findCircleDiagnostics;
    }

    if (m_imageKind == DemoImageKind::EllipseSynthetic) {
        m_findCircleDiagnostics = QStringLiteral("Current image is the ellipse regression image. Click Reset Circle Image before Run FindCircle.");
        setStatus(m_findCircleDiagnostics);
        return m_findCircleDiagnostics;
    }

    QVariantMap calipers = m_display->editableCaliperGeometry(QString::fromLatin1(findCircleCalipersToolId));
    if (!calipers.value(QStringLiteral("valid")).toBool() || calipers.value(QStringLiteral("type")).toString() != QStringLiteral("circle")) {
        createFindCircleCalipers();
        calipers = m_display->editableCaliperGeometry(QString::fromLatin1(findCircleCalipersToolId));
    }

    VisionTools::CircleSearchRegion region;
    region.centerX = calipers.value(QStringLiteral("centerX")).toDouble();
    region.centerY = calipers.value(QStringLiteral("centerY")).toDouble();
    region.radius = calipers.value(QStringLiteral("radius")).toDouble();
    region.searchLength = std::max(1.0, searchLength);
    region.caliperWidth = std::max(1.0, projectionWidth);
    region.startAngleDeg = 0.0;
    region.spanAngleDeg = 360.0;
    region.caliperCount = std::max(3, caliperCount);
    region.searchDirection = searchDirection == 0
        ? VisionTools::CircleSearchDirection::Inward
        : VisionTools::CircleSearchDirection::Outward;

    VisionTools::FindCircleParams params;
    params.caliperParams.sampleCount = 161;
    params.caliperParams.projectionCount = 9;
    params.caliperParams.smoothingSigma = 1.0;
    params.caliperParams.minResponse = std::max(0.0, minResponse);
    params.caliperParams.polarity = edgePolarityFromIndex(polarity);
    params.caliperParams.selection = VisionTools::EdgeSelection::Strongest;
    params.caliperParams.enableSubpixel = true;
    params.fitParams.maxResidual = 3.0;
    params.fitParams.minInlierCount = 12;
    params.fitParams.maxIterations = 20;
    params.fitParams.damping = 1.0e-3;
    params.fitParams.ransacResidual = 4.0;
    params.fitParams.huberDelta = 2.0;
    params.minEdgeResponse = params.caliperParams.minResponse;
    params.circleExpectedPositionTolerance = region.searchLength * 0.5;

    VisionTools::FindCircleTool tool;
    tool.setParams(params);

    QStringList lines;
    int okCount = 0;
    VisionTools::FindCircleResult lastResult;
    lines << QStringLiteral("FindCircle Stability Test");
    for (int i = 0; i < 20; ++i) {
        lastResult = tool.run(VisionTools::ImageView(m_image), region);
        if (lastResult.ok) {
            ++okCount;
        }
        QString line = QStringLiteral("#%1 ok=%2 center=(%3,%4) radius=%5 RMS=%6 score=%7 accepted=%8")
                           .arg(i + 1, 2, 10, QLatin1Char('0'))
                           .arg(lastResult.ok ? QStringLiteral("true") : QStringLiteral("false"))
                           .arg(lastResult.fitResult.circle.centerX, 0, 'f', 3)
                           .arg(lastResult.fitResult.circle.centerY, 0, 'f', 3)
                           .arg(lastResult.fitResult.circle.radius, 0, 'f', 3)
                           .arg(lastResult.fitResult.rmsError, 0, 'f', 6)
                           .arg(lastResult.fitResult.score, 0, 'f', 6)
                           .arg(lastResult.diagnostics.fitInputPointsCount);
        if (!lastResult.ok) {
            line += QStringLiteral(" message=%1").arg(lastResult.message);
        }
        lines << line;
    }
    lines << QStringLiteral("summary okCount=%1/20 finalMessage=%2")
                 .arg(okCount)
                 .arg(lastResult.message.isEmpty() && lastResult.ok ? QStringLiteral("OK") : lastResult.message);
    lines << formatFindCircleDiagnostics(lastResult);

    m_findCircleDiagnostics = lines.join(QLatin1Char('\n'));
    qDebug().noquote() << m_findCircleDiagnostics;

    ToolResultDisplayAdapter::showFindCircle(m_display, QStringLiteral("FindCircle_1"), region, lastResult);
    setStatus(QStringLiteral("FindCircle stability %1/20 OK").arg(okCount));
    return m_findCircleDiagnostics;
}

void IntegratedDemoController::runFindCircleRegression()
{
    runFindCircleRegressionInternal(false, QStringLiteral("Regression"));
}

void IntegratedDemoController::runFindCircleRegressionRobustOff()
{
    runFindCircleRegressionInternal(true, QStringLiteral("Robust Off"));
}

void IntegratedDemoController::runFindCircleRegressionCurrentSettings()
{
    runFindCircleRegressionInternal(false, QStringLiteral("Current Settings"));
}

QVariantMap IntegratedDemoController::runSingleCaliperDebug(double centerX,
                                                            double centerY,
                                                            double searchLength,
                                                            double projectionWidth,
                                                            double searchDirectionAngleDeg,
                                                            int polarity,
                                                            int edgeSelection,
                                                            double minResponse,
                                                            int projectionCount,
                                                            double smoothingSigma,
                                                            double expectedPosition1D,
                                                            double maxPositionDeviation,
                                                            int fallbackEdgeSelection)
{
    QVariantMap output;
    output.insert(QStringLiteral("ok"), false);
    output.insert(QStringLiteral("message"), QString());
    output.insert(QStringLiteral("status"), QString());
    output.insert(QStringLiteral("candidates"), QVariantList());
    output.insert(QStringLiteral("candidatesJson"), QStringLiteral("[]"));
    output.insert(QStringLiteral("profilePeaks"), QVariantList());
    output.insert(QStringLiteral("profilePeaksJson"), QStringLiteral("[]"));
    output.insert(QStringLiteral("selectedIndex"), -1);
    output.insert(QStringLiteral("hasSelected"), false);
    output.insert(QStringLiteral("centerX"), centerX);
    output.insert(QStringLiteral("centerY"), centerY);
    output.insert(QStringLiteral("searchLength"), searchLength);
    output.insert(QStringLiteral("projectionWidth"), projectionWidth);
    output.insert(QStringLiteral("caliperAngleDeg"), searchDirectionAngleDeg - 90.0);
    output.insert(QStringLiteral("searchDirectionAngleDeg"), searchDirectionAngleDeg);

    if (m_image.isNull()) {
        const QString status = QStringLiteral("Image not ready");
        output.insert(QStringLiteral("message"), status);
        output.insert(QStringLiteral("status"), status);
        m_singleCaliperDiagnostics = status;
        output.insert(QStringLiteral("diagnosticsPath"), saveSingleCaliperDiagnosticsToFile());
        setStatus(status);
        return output;
    }

    if (!std::isfinite(centerX) || !std::isfinite(centerY) || !std::isfinite(searchLength)
        || !std::isfinite(projectionWidth) || !std::isfinite(searchDirectionAngleDeg)
        || !std::isfinite(minResponse) || !std::isfinite(smoothingSigma)
        || !std::isfinite(expectedPosition1D) || !std::isfinite(maxPositionDeviation)) {
        const QString status = QStringLiteral("SingleCaliper input contains an invalid number");
        output.insert(QStringLiteral("message"), status);
        output.insert(QStringLiteral("status"), status);
        m_singleCaliperDiagnostics = status;
        output.insert(QStringLiteral("diagnosticsPath"), saveSingleCaliperDiagnosticsToFile());
        setStatus(status);
        return output;
    }

    VisionTools::CaliperRegion region;
    region.centerX = centerX;
    region.centerY = centerY;
    region.length = std::max(1.0, searchLength);
    region.width = std::max(0.0, projectionWidth);
    region.angleDeg = searchDirectionAngleDeg;

    VisionTools::CaliperParams params;
    params.sampleCount = 161;
    params.projectionCount = std::max(1, projectionCount);
    params.smoothingSigma = std::max(0.0, smoothingSigma);
    params.minResponse = std::max(0.0, minResponse);
    params.polarity = edgePolarityFromIndex(polarity);
    params.selection = edgeSelectionFromIndex(edgeSelection);
    params.fallbackSelection = edgeSelectionFromIndex(fallbackEdgeSelection);
    params.expectedPosition1D = expectedPosition1D;
    params.maxPositionDeviation = std::max(0.0, maxPositionDeviation);
    params.allowFallbackSelection = false;
    params.enableSubpixel = true;

    VisionTools::CaliperTool tool;
    tool.setParams(params);
    const VisionTools::CaliperResult result = tool.run(VisionTools::ImageView(m_image), region);

    const double angleRad = region.angleDeg * pi / 180.0;
    const double dirX = std::cos(angleRad);
    const double dirY = std::sin(angleRad);
    const double tanX = -dirY;
    const double tanY = dirX;
    const bool hasSelected = !result.selectedEdges.empty() && isFiniteEdgePoint(result.selectedEdges.front());
    const VisionTools::EdgePoint selected = hasSelected ? result.selectedEdges.front() : VisionTools::EdgePoint {};
    const int bestIndex = selectedEdgeIndex(result.candidates, result.selectedEdges);
    const bool selectedWithinExpected = hasSelected
        && std::abs(selected.position1D - params.expectedPosition1D) <= params.maxPositionDeviation;
    const QPointF expectedEdge(region.centerX + params.expectedPosition1D * dirX,
                               region.centerY + params.expectedPosition1D * dirY);

    const QString polarityText = edgePolarityName(params.polarity);
    const QString edgeSelectionText = edgeSelectionName(params.selection);
    const QString fallbackEdgeSelectionText = edgeSelectionName(params.fallbackSelection);

    QJsonArray candidatePointsJson;
    QVariantList candidatePoints;
    int filteredSelectedIndex = -1;
    for (size_t i = 0; i < result.candidates.size(); ++i) {
        if (!isFiniteEdgePoint(result.candidates[i])) {
            continue;
        }
        const bool selectedCandidate = hasSelected && static_cast<int>(i) == bestIndex;
        if (selectedCandidate) {
            filteredSelectedIndex = candidatePoints.size();
        }
        const bool withinExpected = std::abs(result.candidates[i].position1D - params.expectedPosition1D)
            <= params.maxPositionDeviation;
        const int filteredIndex = candidatePoints.size();
        candidatePoints.append(edgePointMap(result.candidates[i],
                                            filteredIndex,
                                            selectedCandidate,
                                            polarityText,
                                            withinExpected));
        candidatePointsJson.append(edgePointJson(result.candidates[i],
                                                 filteredIndex,
                                                 selectedCandidate,
                                                 polarityText,
                                                 withinExpected));
    }

    std::vector<double> responseArray;
    responseArray.reserve(result.gradient.size());
    for (double gradient : result.gradient) {
        responseArray.push_back(polarityResponse(gradient, params.polarity));
    }

    QJsonArray profilePeaksJson;
    QVariantList profilePeaks;
    const double step = result.smoothedProfile.size() > 1
        ? region.length / static_cast<double>(result.smoothedProfile.size() - 1)
        : 0.0;
    for (int i = 1; i + 1 < static_cast<int>(responseArray.size()); ++i) {
        const double response = responseArray[static_cast<size_t>(i)];
        if (response < responseArray[static_cast<size_t>(i - 1)]
            || response < responseArray[static_cast<size_t>(i + 1)]) {
            continue;
        }

        const double y0 = responseArray[static_cast<size_t>(i - 1)];
        const double y1 = response;
        const double y2 = responseArray[static_cast<size_t>(i + 1)];
        double delta = 0.0;
        const double denominator = y0 - 2.0 * y1 + y2;
        if (std::abs(denominator) > 1e-9) {
            delta = 0.5 * (y0 - y2) / denominator;
            delta = std::clamp(delta, -0.5, 0.5);
        }
        const double subIndex = static_cast<double>(i) + delta;
        const double position1D = -region.length * 0.5 + subIndex * step;
        const bool aboveMinResponse = response >= params.minResponse;
        profilePeaks.append(profilePeakMap(i,
                                           subIndex,
                                           position1D,
                                           response,
                                           result.gradient[static_cast<size_t>(i)],
                                           aboveMinResponse));
        profilePeaksJson.append(profilePeakJson(i,
                                                subIndex,
                                                position1D,
                                                response,
                                                result.gradient[static_cast<size_t>(i)],
                                                aboveMinResponse));
    }

    const int peakIndex = hasSelected ? static_cast<int>(std::round(selected.subIndex)) : -1;
    const double subpixelDelta = hasSelected ? selected.subIndex - static_cast<double>(peakIndex) : 0.0;

    QStringList lines;
    lines << QStringLiteral("Single Caliper Debug");
    lines << QStringLiteral("centerX=%1").arg(region.centerX, 0, 'f', 3);
    lines << QStringLiteral("centerY=%1").arg(region.centerY, 0, 'f', 3);
    lines << QStringLiteral("searchLength=%1").arg(region.length, 0, 'f', 3);
    lines << QStringLiteral("projectionWidth=%1").arg(region.width, 0, 'f', 3);
    lines << QStringLiteral("searchDirectionAngleDeg=%1").arg(region.angleDeg, 0, 'f', 3);
    lines << QStringLiteral("dirX=%1").arg(dirX, 0, 'f', 6);
    lines << QStringLiteral("dirY=%1").arg(dirY, 0, 'f', 6);
    lines << QStringLiteral("tanX=%1").arg(tanX, 0, 'f', 6);
    lines << QStringLiteral("tanY=%1").arg(tanY, 0, 'f', 6);
    lines << QStringLiteral("polarity=%1").arg(polarityText);
    lines << QStringLiteral("edgeSelection=%1").arg(edgeSelectionText);
    lines << QStringLiteral("fallbackEdgeSelection=%1").arg(fallbackEdgeSelectionText);
    lines << QStringLiteral("allowFallbackSelection=%1").arg(params.allowFallbackSelection ? QStringLiteral("true") : QStringLiteral("false"));
    lines << QStringLiteral("minResponse=%1").arg(params.minResponse, 0, 'f', 3);
    lines << QStringLiteral("projectionCount=%1").arg(params.projectionCount);
    lines << QStringLiteral("smoothingSigma=%1").arg(params.smoothingSigma, 0, 'f', 3);
    lines << QStringLiteral("expectedPosition1D=%1").arg(params.expectedPosition1D, 0, 'f', 3);
    lines << QStringLiteral("maxPositionDeviation=%1").arg(params.maxPositionDeviation, 0, 'f', 3);
    lines << QStringLiteral("ok=%1").arg(result.ok ? QStringLiteral("true") : QStringLiteral("false"));
    lines << QStringLiteral("message=%1").arg(result.message);
    lines << QStringLiteral("candidateCount=%1").arg(static_cast<int>(candidatePoints.size()));
    lines << QStringLiteral("selectedIndex=%1").arg(filteredSelectedIndex);
    lines << QStringLiteral("selectedWithinExpected=%1").arg(selectedWithinExpected ? QStringLiteral("true") : QStringLiteral("false"));
    lines << QStringLiteral("selectedByFallback=%1").arg(result.selectedByFallback ? QStringLiteral("true") : QStringLiteral("false"));
    lines << QStringLiteral("expected edge x=%1").arg(expectedEdge.x(), 0, 'f', 6);
    lines << QStringLiteral("expected edge y=%1").arg(expectedEdge.y(), 0, 'f', 6);
    lines << QStringLiteral("selected edge x=%1").arg(hasSelected ? selected.x : 0.0, 0, 'f', 6);
    lines << QStringLiteral("selected edge y=%1").arg(hasSelected ? selected.y : 0.0, 0, 'f', 6);
    lines << QStringLiteral("response=%1").arg(hasSelected ? selected.response : 0.0, 0, 'f', 6);
    lines << QStringLiteral("position1D=%1").arg(hasSelected ? selected.position1D : 0.0, 0, 'f', 6);
    lines << QStringLiteral("subIndex=%1").arg(hasSelected ? selected.subIndex : 0.0, 0, 'f', 6);
    lines << QStringLiteral("peakIndex=%1").arg(peakIndex);
    lines << QStringLiteral("subpixelDelta=%1").arg(subpixelDelta, 0, 'f', 6);
    lines << QStringLiteral("profile=%1").arg(formatDoubleArray(result.profile));
    lines << QStringLiteral("smoothedProfile=%1").arg(formatDoubleArray(result.smoothedProfile));
    lines << QStringLiteral("gradient=%1").arg(formatDoubleArray(result.gradient));
    lines << QStringLiteral("response array=%1").arg(formatDoubleArray(responseArray));
    lines << QStringLiteral("profile peaks:");
    for (const QVariant& value : profilePeaks) {
        const QVariantMap peak = value.toMap();
        lines << QStringLiteral("#%1 subIndex=%2 position1D=%3 response=%4 gradient=%5 aboveMinResponse=%6")
                     .arg(peak.value(QStringLiteral("index")).toInt())
                     .arg(peak.value(QStringLiteral("subIndex")).toDouble(), 0, 'f', 6)
                     .arg(peak.value(QStringLiteral("position1D")).toDouble(), 0, 'f', 6)
                     .arg(peak.value(QStringLiteral("response")).toDouble(), 0, 'f', 6)
                     .arg(peak.value(QStringLiteral("gradient")).toDouble(), 0, 'f', 6)
                     .arg(peak.value(QStringLiteral("aboveMinResponse")).toBool() ? QStringLiteral("true") : QStringLiteral("false"));
    }
    lines << QStringLiteral("candidates:");

    for (size_t i = 0; i < result.candidates.size(); ++i) {
        const VisionTools::EdgePoint& edge = result.candidates[i];
        if (!isFiniteEdgePoint(edge)) {
            lines << QStringLiteral("#%1 invalid non-finite candidate skipped").arg(static_cast<int>(i));
            continue;
        }
        const int candidatePeakIndex = static_cast<int>(std::round(edge.subIndex));
        const bool selectedCandidate = hasSelected && static_cast<int>(i) == bestIndex;
        const bool withinExpected = std::abs(edge.position1D - params.expectedPosition1D) <= params.maxPositionDeviation;
        lines << QStringLiteral("#%1 x=%2 y=%3 position1D=%4 subIndex=%5 response=%6 polarity=%7 selected=%8 withinExpected=%9 peakIndex=%10 subpixelDelta=%11")
                     .arg(static_cast<int>(i))
                     .arg(edge.x, 0, 'f', 6)
                     .arg(edge.y, 0, 'f', 6)
                     .arg(edge.position1D, 0, 'f', 6)
                     .arg(edge.subIndex, 0, 'f', 6)
                     .arg(edge.response, 0, 'f', 6)
                     .arg(polarityText)
                     .arg(selectedCandidate ? QStringLiteral("true") : QStringLiteral("false"))
                     .arg(withinExpected ? QStringLiteral("true") : QStringLiteral("false"))
                     .arg(candidatePeakIndex)
                     .arg(edge.subIndex - static_cast<double>(candidatePeakIndex), 0, 'f', 6);
    }

    m_singleCaliperDiagnostics = lines.join(QLatin1Char('\n'));
    qDebug().noquote() << m_singleCaliperDiagnostics;
    const QString diagnosticsPath = saveSingleCaliperDiagnosticsToFile();
    if (!diagnosticsPath.isEmpty()) {
        m_singleCaliperDiagnostics.append(QStringLiteral("\nSavedTo=%1").arg(diagnosticsPath));
    }

    const QString status = QStringLiteral("SingleCaliper %1 candidates=%2 selected=(%3,%4) response=%5 pos1D=%6")
                               .arg(result.ok ? QStringLiteral("OK") : QStringLiteral("NG"))
                               .arg(static_cast<int>(result.candidates.size()))
                               .arg(hasSelected ? selected.x : 0.0, 0, 'f', 3)
                               .arg(hasSelected ? selected.y : 0.0, 0, 'f', 3)
                               .arg(hasSelected ? selected.response : 0.0, 0, 'f', 3)
                               .arg(hasSelected ? selected.position1D : 0.0, 0, 'f', 3);
    setStatus(status);

    output.insert(QStringLiteral("ok"), result.ok);
    output.insert(QStringLiteral("message"), result.message);
    output.insert(QStringLiteral("status"), status);
    output.insert(QStringLiteral("diagnosticsPath"), diagnosticsPath);
    output.insert(QStringLiteral("centerX"), region.centerX);
    output.insert(QStringLiteral("centerY"), region.centerY);
    output.insert(QStringLiteral("searchLength"), region.length);
    output.insert(QStringLiteral("projectionWidth"), region.width);
    output.insert(QStringLiteral("caliperAngleDeg"), region.angleDeg - 90.0);
    output.insert(QStringLiteral("searchDirectionAngleDeg"), region.angleDeg);
    output.insert(QStringLiteral("polarity"), polarityText);
    output.insert(QStringLiteral("edgeSelection"), edgeSelectionText);
    output.insert(QStringLiteral("fallbackEdgeSelection"), fallbackEdgeSelectionText);
    output.insert(QStringLiteral("allowFallbackSelection"), params.allowFallbackSelection);
    output.insert(QStringLiteral("selectedByFallback"), result.selectedByFallback);
    output.insert(QStringLiteral("selectedWithinExpected"), selectedWithinExpected);
    output.insert(QStringLiteral("minResponse"), params.minResponse);
    output.insert(QStringLiteral("projectionCount"), params.projectionCount);
    output.insert(QStringLiteral("smoothingSigma"), params.smoothingSigma);
    output.insert(QStringLiteral("expectedPosition1D"), params.expectedPosition1D);
    output.insert(QStringLiteral("maxPositionDeviation"), params.maxPositionDeviation);
    output.insert(QStringLiteral("candidateCount"), static_cast<int>(candidatePoints.size()));
    output.insert(QStringLiteral("selectedIndex"), filteredSelectedIndex);
    output.insert(QStringLiteral("hasSelected"), hasSelected && filteredSelectedIndex >= 0);
    output.insert(QStringLiteral("selectedX"), hasSelected ? selected.x : 0.0);
    output.insert(QStringLiteral("selectedY"), hasSelected ? selected.y : 0.0);
    output.insert(QStringLiteral("expectedX"), expectedEdge.x());
    output.insert(QStringLiteral("expectedY"), expectedEdge.y());
    output.insert(QStringLiteral("selectedResponse"), hasSelected ? selected.response : 0.0);
    output.insert(QStringLiteral("selectedPosition1D"), hasSelected ? selected.position1D : 0.0);
    output.insert(QStringLiteral("candidates"), candidatePoints);
    output.insert(QStringLiteral("candidatesJson"), QString::fromUtf8(QJsonDocument(candidatePointsJson).toJson(QJsonDocument::Compact)));
    output.insert(QStringLiteral("profilePeaks"), profilePeaks);
    output.insert(QStringLiteral("profilePeaksJson"), QString::fromUtf8(QJsonDocument(profilePeaksJson).toJson(QJsonDocument::Compact)));
    return output;
}

void IntegratedDemoController::clearSingleCaliperDebugGraphics()
{
    if (!m_display) {
        return;
    }
    m_display->clearToolGraphics(QString::fromLatin1(singleCaliperToolId));
    m_singleCaliperDiagnostics.clear();
    setStatus(QStringLiteral("Single Caliper debug graphics cleared"));
}

QString IntegratedDemoController::lastSingleCaliperDiagnostics() const
{
    return m_singleCaliperDiagnostics;
}

QString IntegratedDemoController::lastFindLineDiagnostics() const
{
    return m_findLineDiagnostics;
}

QString IntegratedDemoController::lastFindCircleDiagnostics() const
{
    return m_findCircleDiagnostics;
}

void IntegratedDemoController::runFindEllipse()
{
    runFindEllipseWithLoss(QStringLiteral("Huber"));
}

void IntegratedDemoController::runFindEllipseWithLoss(const QString& lossType)
{
    if (!m_display) {
        setStatus(QStringLiteral("Display not ready"));
        return;
    }

    const VisionTools::RobustLossType robustLoss = robustLossFromName(lossType);
    const QString robustName = robustLossName(robustLoss);

    m_image = createEllipseImage();
    m_imageKind = DemoImageKind::EllipseSynthetic;
    m_display->setKeepViewTransformOnNewImage(false);
    m_display->setAutoFitOnNewImage(true);
    m_display->setImage(m_image);
    m_display->fitToWindow();
    m_display->clearAllToolGraphics();

    VisionTools::FindEllipseParams params;
    params.centerX = ellipseCenterX - 2.0;
    params.centerY = ellipseCenterY + 2.0;
    params.radiusA = ellipseRadiusA - 4.0;
    params.radiusB = ellipseRadiusB + 2.0;
    params.angleDeg = ellipseAngleDeg - 2.0;
    params.startAngleDeg = 0.0;
    params.spanAngleDeg = 360.0;
    params.caliperCount = 48;
    params.searchLength = 75.0;
    params.projectionWidth = 12.0;
    params.polarity = VisionTools::EdgePolarity::DarkToLight;
    params.selection = VisionTools::EdgeSelection::Strongest;
    params.minResponse = 5.0;
    params.maxResidual = 3.0;
    params.maxIterations = 35;
    params.searchDirection = VisionTools::EllipseSearchDirection::FromOutsideToInside;
    params.enableRobustFiltering = false;
    params.robust.lossType = robustLoss;
    params.robust.huberDelta = 2.0;
    params.robust.tukeyC = 4.685;
    params.robust.irlsIterations = 12;
    params.robust.minWeight = 1.0e-6;

    VisionTools::FindEllipseTool tool;
    tool.setParams(params);
    const VisionTools::FindEllipseResult result = tool.run(VisionTools::ImageView(m_image));

    ToolResultDisplayAdapter::showFindEllipse(m_display, QStringLiteral("FindEllipse_1"), result);
    double minWeight = 1.0;
    for (double weight : result.pointWeights) {
        minWeight = std::min(minWeight, weight);
    }
    setStatus(QStringLiteral("FindEllipse_1 %1 loss=%2 A=%3 B=%4 angle=%5 RMS=%6 Max=%7 InlierRatio=%8 In=%9 Out=%10 minW=%11")
                  .arg(result.ok ? QStringLiteral("OK") : QStringLiteral("NG"))
                  .arg(robustName)
                  .arg(result.fittedEllipse.radiusA, 0, 'f', 2)
                  .arg(result.fittedEllipse.radiusB, 0, 'f', 2)
                  .arg(result.fittedEllipse.angleDeg, 0, 'f', 2)
                  .arg(result.diagnostics.rmsError, 0, 'f', 3)
                  .arg(result.diagnostics.maxError, 0, 'f', 3)
                  .arg(result.diagnostics.inlierRatio, 0, 'f', 2)
                  .arg(result.diagnostics.inlierCount)
                  .arg(result.diagnostics.outlierCount)
                  .arg(minWeight, 0, 'g', 3));
}

void IntegratedDemoController::clearToolGraphics()
{
    if (!m_display) {
        return;
    }
    m_display->clearAllToolGraphics();
    setStatus(QStringLiteral("Tool graphics cleared"));
}

void IntegratedDemoController::clearCaliperGraphics()
{
    if (!m_display) {
        return;
    }
    m_display->clearToolGraphics(QStringLiteral("Caliper_1"));
    setStatus(QStringLiteral("Caliper_1 graphics cleared"));
}

void IntegratedDemoController::clearFindLineGraphics()
{
    if (!m_display) {
        return;
    }
    m_display->clearToolGraphics(QStringLiteral("FindLine_1"));
    setStatus(QStringLiteral("FindLine_1 graphics cleared"));
}

void IntegratedDemoController::clearFindCircleGraphics()
{
    if (!m_display) {
        return;
    }
    m_display->clearToolGraphics(QStringLiteral("FindCircle_1"));
    setStatus(QStringLiteral("FindCircle_1 graphics cleared"));
}

void IntegratedDemoController::clearFindEllipseGraphics()
{
    if (!m_display) {
        return;
    }
    m_display->clearToolGraphics(QStringLiteral("FindEllipse_1"));
    setStatus(QStringLiteral("FindEllipse_1 graphics cleared"));
}

bool IntegratedDemoController::buildGrayTemplateFromCurrentImage(double x, double y, double width, double height)
{
    if (m_image.isNull()) {
        m_grayMatchDiagnostics = QStringLiteral("GrayTemplate build failed: image not ready");
        setStatus(m_grayMatchDiagnostics);
        return false;
    }

    const QRect roi(static_cast<int>(std::round(x)),
                    static_cast<int>(std::round(y)),
                    static_cast<int>(std::round(width)),
                    static_cast<int>(std::round(height)));
    VisionTools::Matching::GrayMatcher matcher;
    m_grayTemplate = matcher.buildTemplate(m_image, roi);
    m_grayTemplateRoi = roi.normalized().intersected(QRect(0, 0, m_image.width(), m_image.height()));

    const bool ok = m_grayTemplate.isValid();
    QStringList lines;
    lines << QStringLiteral("GrayTemplate");
    lines << QStringLiteral("ok=%1").arg(ok ? QStringLiteral("true") : QStringLiteral("false"));
    lines << QStringLiteral("roi=(%1,%2,%3,%4)")
                 .arg(m_grayTemplateRoi.x())
                 .arg(m_grayTemplateRoi.y())
                 .arg(m_grayTemplateRoi.width())
                 .arg(m_grayTemplateRoi.height());
    lines << QStringLiteral("width=%1").arg(m_grayTemplate.width);
    lines << QStringLiteral("height=%1").arg(m_grayTemplate.height);
    lines << QStringLiteral("origin=(%1,%2)").arg(m_grayTemplate.originX, 0, 'f', 3).arg(m_grayTemplate.originY, 0, 'f', 3);
    lines << QStringLiteral("mean=%1").arg(m_grayTemplate.mean, 0, 'f', 6);
    lines << QStringLiteral("norm=%1").arg(m_grayTemplate.norm, 0, 'f', 6);
    lines << QStringLiteral("message=%1").arg(m_grayTemplate.message);
    m_grayMatchDiagnostics = lines.join(QLatin1Char('\n'));

    if (m_display) {
        m_display->clearToolGraphics(QString::fromLatin1(grayMatchToolId));
        m_display->addToolRect(QString::fromLatin1(grayMatchToolId),
                               QStringLiteral("template_roi"),
                               m_grayTemplateRoi.x(),
                               m_grayTemplateRoi.y(),
                               m_grayTemplateRoi.width(),
                               m_grayTemplateRoi.height(),
                               80,
                               220,
                               120);
        m_display->addToolStatusText(QString::fromLatin1(grayMatchToolId),
                                     m_grayTemplateRoi.x(),
                                     m_grayTemplateRoi.y() - 20.0,
                                     ok ? QStringLiteral("GrayTemplate OK") : m_grayTemplate.message,
                                     ok);
    }

    setStatus(ok ? QStringLiteral("Gray template built") : m_grayTemplate.message);
    return ok;
}

QVariantMap IntegratedDemoController::runGrayMatch(double searchX,
                                                   double searchY,
                                                   double searchWidth,
                                                   double searchHeight,
                                                   double minScore,
                                                   int stepX,
                                                   int stepY,
                                                   int topK,
                                                   bool enableSubpixelRefine)
{
    QVariantMap output;
    if (m_image.isNull()) {
        m_grayMatchDiagnostics = QStringLiteral("GrayMatch failed: image not ready");
        setStatus(m_grayMatchDiagnostics);
        output.insert(QStringLiteral("ok"), false);
        output.insert(QStringLiteral("message"), m_grayMatchDiagnostics);
        return output;
    }
    if (!m_grayTemplate.isValid()) {
        m_grayMatchDiagnostics = QStringLiteral("GrayMatch failed: build a valid template first");
        setStatus(m_grayMatchDiagnostics);
        output.insert(QStringLiteral("ok"), false);
        output.insert(QStringLiteral("message"), m_grayMatchDiagnostics);
        return output;
    }

    VisionTools::Matching::GrayMatchParams params;
    params.searchRoi = QRect(static_cast<int>(std::round(searchX)),
                             static_cast<int>(std::round(searchY)),
                             static_cast<int>(std::round(searchWidth)),
                             static_cast<int>(std::round(searchHeight)));
    params.minScore = std::clamp(minScore, -1.0, 1.0);
    params.pyramidLevels = 1;
    params.stepX = std::max(1, stepX);
    params.stepY = std::max(1, stepY);
    params.topK = std::max(1, topK);
    params.enableSubpixelRefine = enableSubpixelRefine;

    VisionTools::Matching::GrayMatcher matcher;
    const VisionTools::Matching::GrayMatchResult result = matcher.match(m_image, m_grayTemplate, params);

    QStringList lines;
    lines << QStringLiteral("GrayMatch");
    lines << QStringLiteral("ok=%1").arg(result.ok ? QStringLiteral("true") : QStringLiteral("false"));
    lines << QStringLiteral("x=%1").arg(result.x, 0, 'f', 3);
    lines << QStringLiteral("y=%1").arg(result.y, 0, 'f', 3);
    lines << QStringLiteral("centerX=%1").arg(result.x + m_grayTemplate.originX, 0, 'f', 3);
    lines << QStringLiteral("centerY=%1").arg(result.y + m_grayTemplate.originY, 0, 'f', 3);
    lines << QStringLiteral("score=%1").arg(result.score, 0, 'f', 6);
    lines << QStringLiteral("candidateCount=%1").arg(static_cast<int>(result.candidates.size()));
    lines << QStringLiteral("message=%1").arg(result.message);
    for (size_t i = 0; i < result.candidates.size(); ++i) {
        const auto& candidate = result.candidates[i];
        lines << QStringLiteral("candidate[%1] x=%2 y=%3 score=%4")
                     .arg(static_cast<int>(i))
                     .arg(candidate.x, 0, 'f', 3)
                     .arg(candidate.y, 0, 'f', 3)
                     .arg(candidate.score, 0, 'f', 6);
    }
    m_grayMatchDiagnostics = lines.join(QLatin1Char('\n'));

    if (m_display) {
        const QRect searchRoi = params.searchRoi.normalized().intersected(QRect(0, 0, m_image.width(), m_image.height()));
        m_display->clearToolGraphics(QString::fromLatin1(grayMatchToolId));
        m_display->addToolRect(QString::fromLatin1(grayMatchToolId),
                               QStringLiteral("template_roi"),
                               m_grayTemplateRoi.x(),
                               m_grayTemplateRoi.y(),
                               m_grayTemplateRoi.width(),
                               m_grayTemplateRoi.height(),
                               80,
                               220,
                               120);
        m_display->addToolRect(QString::fromLatin1(grayMatchToolId),
                               QStringLiteral("search_roi"),
                               searchRoi.x(),
                               searchRoi.y(),
                               searchRoi.width(),
                               searchRoi.height(),
                               80,
                               160,
                               255);
        if (!result.candidates.empty()) {
            m_display->addToolRect(QString::fromLatin1(grayMatchToolId),
                                   QStringLiteral("result_rect"),
                                   result.x,
                                   result.y,
                                   m_grayTemplate.width,
                                   m_grayTemplate.height,
                                   255,
                                   210,
                                   80);
            m_display->addToolPointMarker(QString::fromLatin1(grayMatchToolId),
                                          QStringLiteral("center"),
                                          result.x + m_grayTemplate.originX,
                                          result.y + m_grayTemplate.originY,
                                          40,
                                          255,
                                          120,
                                          15.0);
        }
        m_display->addToolStatusText(QString::fromLatin1(grayMatchToolId),
                                     result.x,
                                     result.y - 24.0,
                                     QStringLiteral("GrayMatch %1 Score=%2")
                                         .arg(result.ok ? QStringLiteral("OK") : QStringLiteral("NG"))
                                         .arg(result.score, 0, 'f', 3),
                                     result.ok);
    }

    output.insert(QStringLiteral("ok"), result.ok);
    output.insert(QStringLiteral("x"), result.x);
    output.insert(QStringLiteral("y"), result.y);
    output.insert(QStringLiteral("score"), result.score);
    output.insert(QStringLiteral("message"), result.message);
    setStatus(QStringLiteral("GrayMatch %1 score=%2")
                  .arg(result.ok ? QStringLiteral("OK") : QStringLiteral("NG"))
                  .arg(result.score, 0, 'f', 3));
    return output;
}

QString IntegratedDemoController::lastGrayMatchDiagnostics() const
{
    return m_grayMatchDiagnostics;
}

void IntegratedDemoController::clearGrayMatchGraphics()
{
    if (m_display) {
        m_display->clearToolGraphics(QString::fromLatin1(grayMatchToolId));
    }
    m_grayMatchDiagnostics.clear();
    setStatus(QStringLiteral("GrayMatch graphics cleared"));
}

QString IntegratedDemoController::runDependencySelfTest()
{
    const QString diagnostics = VisionTools::Backend::DependencySelfTest::run();
    qDebug().noquote() << diagnostics;
    setStatus(QStringLiteral("Dependency self test completed"));
    return diagnostics;
}

QImage IntegratedDemoController::createSyntheticImage() const
{
    constexpr int width = 800;
    constexpr int height = 600;
    QImage image(width, height, QImage::Format_Grayscale8);

    const double lineAngleRad = lineAngleDeg * pi / 180.0;
    const double nx = -std::sin(lineAngleRad);
    const double ny = std::cos(lineAngleRad);

    std::mt19937 rng(42);
    std::uniform_int_distribution<int> noise(-4, 4);

    for (int y = 0; y < height; ++y) {
        uchar* row = image.scanLine(y);
        for (int x = 0; x < width; ++x) {
            const double lineDistance = (x - lineCenterX) * nx + (y - lineCenterY) * ny;
            int value = lineDistance < 0.0 ? 45 : 180;

            const double dx = x - circleCenterX;
            const double dy = y - circleCenterY;
            const double circleDistance = std::sqrt(dx * dx + dy * dy);
            if (circleDistance < circleRadius) {
                value = 55;
            }

            row[x] = static_cast<uchar>(std::clamp(value + noise(rng), 0, 255));
        }
    }

    return image;
}

QImage IntegratedDemoController::createCircleRegressionImage() const
{
    constexpr int width = 800;
    constexpr int height = 600;
    QImage image(width, height, QImage::Format_Grayscale8);

    for (int y = 0; y < height; ++y) {
        uchar* row = image.scanLine(y);
        for (int x = 0; x < width; ++x) {
            const double dx = x - regressionCircleCenterX;
            const double dy = y - regressionCircleCenterY;
            const double distance = std::sqrt(dx * dx + dy * dy);
            row[x] = distance <= regressionCircleRadius ? 200 : 50;
        }
    }

    return image;
}

QImage IntegratedDemoController::createEllipseImage() const
{
    constexpr int width = 800;
    constexpr int height = 600;
    QImage image(width, height, QImage::Format_Grayscale8);

    const double theta = ellipseAngleDeg * pi / 180.0;
    const double c = std::cos(theta);
    const double s = std::sin(theta);

    std::mt19937 rng(31);
    std::uniform_int_distribution<int> noise(-4, 4);

    for (int y = 0; y < height; ++y) {
        uchar* row = image.scanLine(y);
        for (int x = 0; x < width; ++x) {
            const double dx = x - ellipseCenterX;
            const double dy = y - ellipseCenterY;
            const double xLocal = c * dx + s * dy;
            const double yLocal = -s * dx + c * dy;
            const double implicit = xLocal * xLocal / (ellipseRadiusA * ellipseRadiusA)
                                  + yLocal * yLocal / (ellipseRadiusB * ellipseRadiusB);
            int value = implicit <= 1.0 ? 200 : 50;

            const bool gap = implicit > 0.96 && implicit < 1.05 && xLocal > -80.0 && xLocal < -35.0 && yLocal > 55.0;
            if (gap) {
                value = 50;
            }

            const bool distractor = implicit > 1.15 && implicit < 1.22 && xLocal > 60.0 && xLocal < 120.0 && yLocal < -20.0;
            if (distractor) {
                value = 200;
            }

            const bool strongOutlierEdge = std::abs(xLocal - ellipseRadiusA - 22.0) < 2.5 && yLocal > -60.0 && yLocal < 55.0;
            if (strongOutlierEdge) {
                value = 200;
            }

            row[x] = static_cast<uchar>(std::clamp(value + noise(rng), 0, 255));
        }
    }

    return image;
}

QString IntegratedDemoController::localFilePath(const QString& pathOrUrl) const
{
    const QUrl url(pathOrUrl);
    if (url.isLocalFile()) {
        return url.toLocalFile();
    }
    return pathOrUrl;
}

void IntegratedDemoController::createSyntheticCircleCalipers()
{
    if (!m_display) {
        return;
    }

    m_display->createEditableCircleCalipers(QString::fromLatin1(findCircleCalipersToolId),
                                            circleCenterX,
                                            circleCenterY,
                                            circleRadius,
                                            48.0,
                                            120.0,
                                            48);
}

void IntegratedDemoController::createRegressionCircleCalipers()
{
    if (!m_display) {
        return;
    }

    m_display->createEditableCircleCalipers(QString::fromLatin1(findCircleCalipersToolId),
                                            regressionCircleCenterX,
                                            regressionCircleCenterY,
                                            regressionCircleRadius,
                                            8.0,
                                            40.0,
                                            36);
}

void IntegratedDemoController::runFindCircleRegressionInternal(bool robustOff, const QString& label)
{
    if (!m_display) {
        setStatus(QStringLiteral("Display not ready"));
        return;
    }

    m_image = createCircleRegressionImage();
    m_imageKind = DemoImageKind::CircleSynthetic;
    m_display->setKeepViewTransformOnNewImage(false);
    m_display->setAutoFitOnNewImage(true);
    m_display->setImage(m_image);
    m_display->fitToWindow();
    m_display->clearAllToolGraphics();
    createRegressionCircleCalipers();

    VisionTools::CircleSearchRegion region;
    region.centerX = regressionCircleCenterX;
    region.centerY = regressionCircleCenterY;
    region.radius = regressionCircleRadius;
    region.searchLength = 40.0;
    region.caliperWidth = 8.0;
    region.startAngleDeg = 0.0;
    region.spanAngleDeg = 360.0;
    region.caliperCount = 36;
    region.searchDirection = VisionTools::CircleSearchDirection::Outward;

    VisionTools::FindCircleParams params;
    params.caliperParams.sampleCount = 161;
    params.caliperParams.projectionCount = 9;
    params.caliperParams.smoothingSigma = 1.0;
    params.caliperParams.minResponse = 5.0;
    params.caliperParams.polarity = VisionTools::EdgePolarity::Any;
    params.caliperParams.selection = VisionTools::EdgeSelection::NearestToExpected;
    params.caliperParams.expectedPosition1D = 0.0;
    params.caliperParams.maxPositionDeviation = 10.0;
    params.caliperParams.allowFallbackSelection = false;
    params.caliperParams.enableSubpixel = true;
    params.fitParams.maxResidual = 3.0;
    params.fitParams.minInlierCount = 12;
    params.fitParams.maxIterations = 20;
    params.fitParams.damping = 1.0e-3;
    params.fitParams.ransacResidual = 4.0;
    params.fitParams.huberDelta = 2.0;
    if (robustOff) {
        params.fitParams.enableRansac = false;
        params.fitParams.enableHuber = false;
    }
    params.minEdgeResponse = 5.0;

    VisionTools::FindCircleTool tool;
    tool.setParams(params);
    const VisionTools::FindCircleResult result = tool.run(VisionTools::ImageView(m_image), region);

    ToolResultDisplayAdapter::showFindCircle(m_display, QStringLiteral("FindCircle_1"), region, result);

    const double dx = result.fitResult.circle.centerX - regressionCircleCenterX;
    const double dy = result.fitResult.circle.centerY - regressionCircleCenterY;
    const double centerError = std::sqrt(dx * dx + dy * dy);
    const double radiusError = result.fitResult.circle.radius - regressionCircleRadius;

    auto isSamePoint = [](const VisionTools::EdgePoint& a, const VisionTools::EdgePoint& b) {
        return std::abs(a.x - b.x) < 1.0e-6 && std::abs(a.y - b.y) < 1.0e-6;
    };
    auto containsPoint = [&](const std::vector<VisionTools::EdgePoint>& points, const VisionTools::EdgePoint& edge) {
        return std::any_of(points.begin(), points.end(), [&](const VisionTools::EdgePoint& point) {
            return isSamePoint(point, edge);
        });
    };

    QStringList lines;
    lines << QStringLiteral("FindCircle %1").arg(label);
    lines << QStringLiteral("Expected: center=(400.000,300.000), radius=120.000");
    lines << QStringLiteral("Fitted: center=(%1,%2), radius=%3")
                 .arg(result.fitResult.circle.centerX, 0, 'f', 3)
                 .arg(result.fitResult.circle.centerY, 0, 'f', 3)
                 .arg(result.fitResult.circle.radius, 0, 'f', 3);
    lines << QStringLiteral("Error: center=%1 px, dx=%2, dy=%3, radius=%4")
                 .arg(centerError, 0, 'f', 3)
                 .arg(dx, 0, 'f', 3)
                 .arg(dy, 0, 'f', 3)
                 .arg(radiusError, 0, 'f', 3);
    lines << QStringLiteral("Fit: ok=%1, message=%2, rms=%3, max=%4, inliers=%5, outliers=%6, score=%7")
                 .arg(result.ok ? QStringLiteral("true") : QStringLiteral("false"))
                 .arg(result.message)
                 .arg(result.fitResult.rmsError, 0, 'f', 3)
                 .arg(result.fitResult.maxError, 0, 'f', 3)
                 .arg(static_cast<int>(result.fitResult.inlierPoints.size()))
                 .arg(static_cast<int>(result.fitResult.outlierPoints.size()))
                 .arg(result.fitResult.score, 0, 'f', 3);
    lines << QStringLiteral("Params: searchLength=40, projectionWidth=8, polarity=Any, selection=Strongest, searchDirection=Outward, RANSAC=%1, Huber=%2")
                 .arg(params.fitParams.enableRansac ? QStringLiteral("on") : QStringLiteral("off"))
                 .arg(params.fitParams.enableHuber ? QStringLiteral("on") : QStringLiteral("off"));
    lines << QStringLiteral("Calipers:");

    for (size_t i = 0; i < result.calipers.size(); ++i) {
        const VisionTools::CaliperRegion& caliper = result.calipers[i];
        const VisionTools::CaliperResult* caliperResult = i < result.caliperResults.size() ? &result.caliperResults[i] : nullptr;
        const bool valid = caliperResult && caliperResult->ok && !caliperResult->selectedEdges.empty();
        VisionTools::EdgePoint edge;
        if (valid) {
            edge = caliperResult->selectedEdges.front();
        }
        const double residual = valid
            ? std::sqrt((edge.x - result.fitResult.circle.centerX) * (edge.x - result.fitResult.circle.centerX)
                        + (edge.y - result.fitResult.circle.centerY) * (edge.y - result.fitResult.circle.centerY))
                  - result.fitResult.circle.radius
            : 0.0;
        const bool inlier = valid && containsPoint(result.fitResult.inlierPoints, edge);
        const bool outlier = valid && containsPoint(result.fitResult.outlierPoints, edge);
        lines << QStringLiteral("#%1 caliper=(%2,%3) dir=%4 edge=(%5,%6) response=%7 pos1D=%8 residual=%9 valid=%10 inlier=%11 outlier=%12")
                     .arg(static_cast<int>(i), 2)
                     .arg(caliper.centerX, 0, 'f', 3)
                     .arg(caliper.centerY, 0, 'f', 3)
                     .arg(caliper.angleDeg, 0, 'f', 3)
                     .arg(valid ? edge.x : 0.0, 0, 'f', 3)
                     .arg(valid ? edge.y : 0.0, 0, 'f', 3)
                     .arg(valid ? edge.response : 0.0, 0, 'f', 3)
                     .arg(valid ? edge.position1D : 0.0, 0, 'f', 3)
                     .arg(residual, 0, 'f', 3)
                     .arg(valid ? QStringLiteral("true") : QStringLiteral("false"))
                     .arg(inlier ? QStringLiteral("true") : QStringLiteral("false"))
                     .arg(outlier ? QStringLiteral("true") : QStringLiteral("false"));
    }

    m_circleDiagnostics = lines.join(QLatin1Char('\n'));
    qDebug().noquote() << m_circleDiagnostics;

    setStatus(QStringLiteral("%1 %2 centerError=%3 radiusError=%4 rms=%5 in/out=%6/%7")
                  .arg(label)
                  .arg(result.ok ? QStringLiteral("OK") : QStringLiteral("NG"))
                  .arg(centerError, 0, 'f', 3)
                  .arg(radiusError, 0, 'f', 3)
                  .arg(result.fitResult.rmsError, 0, 'f', 3)
                  .arg(static_cast<int>(result.fitResult.inlierPoints.size()))
                  .arg(static_cast<int>(result.fitResult.outlierPoints.size())));
}

QString IntegratedDemoController::saveSingleCaliperDiagnosticsToFile() const
{
    if (m_singleCaliperDiagnostics.isEmpty()) {
        return {};
    }

    QDir outputDir(QCoreApplication::applicationDirPath());
    if (!outputDir.exists(QStringLiteral("single_caliper_debug"))
        && !outputDir.mkpath(QStringLiteral("single_caliper_debug"))) {
        return {};
    }

    if (!outputDir.cd(QStringLiteral("single_caliper_debug"))) {
        return {};
    }

    const QString fileName = QStringLiteral("single_caliper_%1.txt")
                                 .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz")));
    const QString filePath = outputDir.filePath(fileName);

    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        return {};
    }
    if (file.write(m_singleCaliperDiagnostics.toUtf8()) < 0) {
        return {};
    }

    return QDir::toNativeSeparators(filePath);
}

void IntegratedDemoController::setStatus(const QString& status)
{
    if (m_status == status) {
        return;
    }
    m_status = status;
    emit statusChanged();
}
