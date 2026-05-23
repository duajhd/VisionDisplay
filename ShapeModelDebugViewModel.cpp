#include "ShapeModelDebugViewModel.h"

#include "VisionDisplay/VisionDisplayItem.h"
#include "VisionTools/Matching/ShapeModelBuilder.h"
#include "shape_match/coarse/CoarseMatchDebugOverlay.h"
#include "shape_match/coarse/CoarseShapeMatcher.h"
#include "shape_match/core/EdgeImageData.h"
#include "shape_match/io/VisionProGroundTruthReader.h"
#include "shape_match/overlay/VisionDisplayOverlayAdapter.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QImageReader>
#include <QVariantMap>
#include <QtConcurrent/QtConcurrent>

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numeric>

namespace {

constexpr const char* searchRoiId = "search_roi";

QString localPath(const QUrl& url)
{
    return url.isLocalFile() ? url.toLocalFile() : url.toString();
}

cv::Mat grayMatFromQImage(const QImage& image)
{
    if (image.isNull()) {
        return {};
    }
    const QImage gray = image.format() == QImage::Format_Grayscale8
        ? image
        : image.convertToFormat(QImage::Format_Grayscale8);
    cv::Mat mat(gray.height(), gray.width(), CV_8U);
    for (int y = 0; y < gray.height(); ++y) {
        std::memcpy(mat.ptr<uchar>(y), gray.constScanLine(y), static_cast<size_t>(gray.width()));
    }
    return mat;
}

double percentile(std::vector<float> values, double ratio)
{
    if (values.empty()) {
        return 0.0;
    }
    ratio = std::clamp(ratio, 0.0, 1.0);
    const size_t index = static_cast<size_t>(std::round(ratio * static_cast<double>(values.size() - 1)));
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(index), values.end());
    return values[index];
}

ShapeMatch::EdgePolarity convertPolarity(VisionTools::Matching::EdgePolarity polarity)
{
    switch (polarity) {
    case VisionTools::Matching::EdgePolarity::DarkToBright:
        return ShapeMatch::EdgePolarity::DarkToBright;
    case VisionTools::Matching::EdgePolarity::BrightToDark:
        return ShapeMatch::EdgePolarity::BrightToDark;
    case VisionTools::Matching::EdgePolarity::Unknown:
        return ShapeMatch::EdgePolarity::Any;
    }
    return ShapeMatch::EdgePolarity::Any;
}

ShapeMatch::ShapeTemplateModel toCoarseTemplateModel(const VisionTools::Matching::ShapeTemplateModel& model)
{
    ShapeMatch::ShapeTemplateModel out;
    out.templateId = "shape_model_roi";
    out.origin = cv::Point2d(model.originImage.x(), model.originImage.y());
    if (model.levels.empty()) {
        return out;
    }

    const VisionTools::Matching::TemplateLevel& level0 = model.levels.front();
    const std::vector<VisionTools::Matching::TemplatePoint>* sourcePoints = &level0.points;
    if (sourcePoints->empty() && !level0.stablePoints.empty()) {
        sourcePoints = &level0.stablePoints;
    }
    if (sourcePoints->empty() && !level0.allEdgePoints.empty()) {
        sourcePoints = &level0.allEdgePoints;
    }

    out.points.reserve(sourcePoints->size());
    for (const VisionTools::Matching::TemplatePoint& src : *sourcePoints) {
        ShapeMatch::TemplatePoint point;
        point.id = src.id;
        point.position = cv::Point2d(src.modelPos.x(), src.modelPos.y());
        point.normal = ShapeMatch::normalized(cv::Point2d(src.normal.x(), src.normal.y()));
        point.tangent = ShapeMatch::normalized(cv::Point2d(src.tangent.x(), src.tangent.y()));
        point.gradientDir = ShapeMatch::normalized(cv::Point2d(src.gradient.x(), src.gradient.y()), point.normal);
        point.gradientMag = src.gradMag;
        point.weight = std::max(0.0, src.weight);
        point.chainId = src.chainId;
        point.polarity = src.polarityReliable ? convertPolarity(src.polarity) : ShapeMatch::EdgePolarity::Any;
        out.points.push_back(point);
    }
    out.computeBoundingBox();
    return out;
}

bool hasCoarseTemplateSource(const VisionTools::Matching::ShapeTemplateModel& model)
{
    if (model.levels.empty()) {
        return false;
    }
    const VisionTools::Matching::TemplateLevel& level0 = model.levels.front();
    return !level0.points.empty() || !level0.stablePoints.empty() || !level0.allEdgePoints.empty();
}

ShapeMatch::CoarseMatchReport spatiallyFilteredReport(const ShapeMatch::CoarseMatchReport& report,
                                                       int maxCandidates,
                                                       double minFinalScore,
                                                       double minCoverageRatio,
                                                       double minInlierRatio,
                                                       double maxMedianErrorPx,
                                                       double maxP90ErrorPx,
                                                       double minTranslationDistancePx,
                                                       double minAngleDistanceDeg)
{
    ShapeMatch::CoarseMatchReport filtered = report;
    filtered.finalRankedCandidates.clear();
    filtered.finalRankedCandidates.reserve(static_cast<size_t>(std::max(0, maxCandidates)));

    for (const ShapeMatch::ScoredCandidate& candidate : report.finalRankedCandidates) {
        const bool acceptedForDisplay = candidate.score.accepted
            && candidate.score.finalScore >= minFinalScore
            && candidate.score.coverageRatio >= minCoverageRatio
            && candidate.score.inlierRatio >= minInlierRatio
            && candidate.score.medianError <= maxMedianErrorPx
            && candidate.score.p90Error <= maxP90ErrorPx;
        const bool debugGtHit = candidate.poseError.hasGroundTruth && candidate.poseError.poseOk;
        if (!acceptedForDisplay && !debugGtHit) {
            continue;
        }

        bool duplicate = false;
        for (const ShapeMatch::ScoredCandidate& kept : filtered.finalRankedCandidates) {
            const double dx = candidate.pose.x - kept.pose.x;
            const double dy = candidate.pose.y - kept.pose.y;
            const double dxy = std::sqrt(dx * dx + dy * dy);
            const double dtheta = std::abs(ShapeMatch::radToDeg(ShapeMatch::wrapToPi(candidate.pose.theta - kept.pose.theta)));
            if (dxy < minTranslationDistancePx || (dxy < minTranslationDistancePx * 1.5 && dtheta < minAngleDistanceDeg)) {
                duplicate = true;
                break;
            }
        }

        if (duplicate) {
            continue;
        }

        filtered.finalRankedCandidates.push_back(candidate);
        filtered.finalRankedCandidates.back().rank = static_cast<int>(filtered.finalRankedCandidates.size());
        if (static_cast<int>(filtered.finalRankedCandidates.size()) >= maxCandidates) {
            break;
        }
    }
    return filtered;
}

QRect boundedRoiRect(const QRectF& roi, const QSize& imageSize, int minSize)
{
    const QRect imageRect(0, 0, imageSize.width(), imageSize.height());
    QRect rect = roi.normalized().toAlignedRect().intersected(imageRect);
    if (rect.width() < minSize || rect.height() < minSize) {
        return {};
    }
    return rect;
}

void applySearchRoiMask(ShapeMatch::EdgeImageData& data, const QRect& searchRoi)
{
    if (searchRoi.isEmpty() || data.edgeMap.empty()) {
        return;
    }

    const QRect imageRect(0, 0, data.imageSize.width, data.imageSize.height);
    const QRect bounded = searchRoi.intersected(imageRect);
    if (bounded.isEmpty()) {
        data.edgePoints.clear();
        data.edgeNormals.clear();
        data.edgeMap.setTo(0);
        data.gradX.setTo(0.0f);
        data.gradY.setTo(0.0f);
        data.gradMag.setTo(0.0f);
        return;
    }

    std::vector<cv::Point2d> points;
    std::vector<cv::Point2d> normals;
    points.reserve(data.edgePoints.size());
    normals.reserve(data.edgeNormals.size());
    for (size_t i = 0; i < data.edgePoints.size(); ++i) {
        const cv::Point2d p = data.edgePoints[i];
        if (bounded.contains(QPoint(static_cast<int>(std::round(p.x)), static_cast<int>(std::round(p.y))))) {
            points.push_back(p);
            normals.push_back(i < data.edgeNormals.size() ? data.edgeNormals[i] : cv::Point2d(1.0, 0.0));
        }
    }

    for (int y = 0; y < data.imageSize.height; ++y) {
        const bool insideY = y >= bounded.top() && y <= bounded.bottom();
        for (int x = 0; x < data.imageSize.width; ++x) {
            if (insideY && x >= bounded.left() && x <= bounded.right()) {
                continue;
            }
            data.edgeMap.at<uchar>(y, x) = 0;
            data.gradX.at<float>(y, x) = 0.0f;
            data.gradY.at<float>(y, x) = 0.0f;
            data.gradMag.at<float>(y, x) = 0.0f;
        }
    }

    data.edgePoints = std::move(points);
    data.edgeNormals = std::move(normals);
}

ShapeMatch::EdgeImageData edgeDataFromImage(const QImage& image, const QRect& searchRoi = {})
{
    ShapeMatch::EdgeImageData data;
    const cv::Mat gray8 = grayMatFromQImage(image);
    if (gray8.empty()) {
        return data;
    }

    data.imageSize = gray8.size();
    cv::Mat gray32;
    gray8.convertTo(gray32, CV_32F);
    cv::GaussianBlur(gray32, gray32, cv::Size(), 1.0, 1.0, cv::BORDER_REPLICATE);
    cv::Scharr(gray32, data.gradX, CV_32F, 1, 0, 1.0 / 32.0, 0.0, cv::BORDER_REPLICATE);
    cv::Scharr(gray32, data.gradY, CV_32F, 0, 1, 1.0 / 32.0, 0.0, cv::BORDER_REPLICATE);
    cv::magnitude(data.gradX, data.gradY, data.gradMag);

    std::vector<float> magnitudes;
    magnitudes.reserve(static_cast<size_t>(data.gradMag.rows * data.gradMag.cols));
    for (int y = 1; y < data.gradMag.rows - 1; ++y) {
        for (int x = 1; x < data.gradMag.cols - 1; ++x) {
            const float mag = data.gradMag.at<float>(y, x);
            if (mag >= 8.0f) {
                magnitudes.push_back(mag);
            }
        }
    }
    const double threshold = std::max(8.0, percentile(magnitudes, 0.85));
    data.edgeMap = cv::Mat::zeros(data.imageSize, CV_8U);

    for (int y = 1; y < data.gradMag.rows - 1; ++y) {
        for (int x = 1; x < data.gradMag.cols - 1; ++x) {
            const float mag = data.gradMag.at<float>(y, x);
            if (mag < threshold) {
                continue;
            }
            const cv::Point2d normal = ShapeMatch::normalized(cv::Point2d(data.gradX.at<float>(y, x),
                                                                          data.gradY.at<float>(y, x)));
            const double prev = data.gradMag.at<float>(std::clamp(static_cast<int>(std::round(y - normal.y)), 0, data.gradMag.rows - 1),
                                                       std::clamp(static_cast<int>(std::round(x - normal.x)), 0, data.gradMag.cols - 1));
            const double next = data.gradMag.at<float>(std::clamp(static_cast<int>(std::round(y + normal.y)), 0, data.gradMag.rows - 1),
                                                       std::clamp(static_cast<int>(std::round(x + normal.x)), 0, data.gradMag.cols - 1));
            if (mag < prev || mag < next) {
                continue;
            }
            data.edgeMap.at<uchar>(y, x) = 255;
            data.edgePoints.emplace_back(static_cast<double>(x), static_cast<double>(y));
            data.edgeNormals.push_back(normal);
        }
    }
    if (!searchRoi.isEmpty()) {
        applySearchRoiMask(data, searchRoi);
    }
    return data;
}

} // namespace

ShapeModelDebugViewModel::ShapeModelDebugViewModel(QObject* parent)
    : QObject(parent)
{
    connect(&m_coarseWatcher, &QFutureWatcher<CoarseMatchUiResult>::finished, this, [this]() {
        const CoarseMatchUiResult result = m_coarseWatcher.result();
        setCoarseMatchingRunning(false);
        if (!result.ok) {
            setCoarseMatchFinished(false);
            setStatus(result.message);
            updateStatistics();
            return;
        }

        m_lastCoarseSummary = result.message;
        if (m_display) {
            const VisionDisplay::VisionDisplayOverlayData overlay =
                ShapeMatch::VisionDisplayOverlayAdapter::toVisionDisplayOverlay(result.overlay);
            m_display->setOverlayData(QStringLiteral("CoarseMatchOverlay"), overlay);
        }
        setCoarseMatchFinished(true);
        setStatus(result.message);
        updateStatistics();
    });
}

int ShapeModelDebugViewModel::currentLevel() const { return m_currentLevel; }
int ShapeModelDebugViewModel::levelCount() const { return static_cast<int>(m_model.levels.size()); }
bool ShapeModelDebugViewModel::showAllEdges() const { return m_options.showAllEdges; }
bool ShapeModelDebugViewModel::showStablePoints() const { return m_options.showStablePoints; }
bool ShapeModelDebugViewModel::showChains() const { return m_options.showChains; }
bool ShapeModelDebugViewModel::showSampledPoints() const { return m_options.showSampledPoints; }
bool ShapeModelDebugViewModel::showNormals() const { return m_options.showNormals; }
bool ShapeModelDebugViewModel::showTangents() const { return m_options.showTangents; }
bool ShapeModelDebugViewModel::showCurvature() const { return m_options.showCurvature; }
bool ShapeModelDebugViewModel::showRejectedPoints() const { return m_options.showRejectedPoints; }
bool ShapeModelDebugViewModel::showChainId() const { return m_options.showChainId; }
bool ShapeModelDebugViewModel::showPointId() const { return m_options.showPointId; }
int ShapeModelDebugViewModel::normalStep() const { return m_options.normalStep; }
double ShapeModelDebugViewModel::normalLength() const { return m_options.normalLength; }
int ShapeModelDebugViewModel::tangentStep() const { return m_options.tangentStep; }
double ShapeModelDebugViewModel::tangentLength() const { return m_options.tangentLength; }
QRectF ShapeModelDebugViewModel::modelRoi() const { return m_modelRoi; }
bool ShapeModelDebugViewModel::modelRoiVisible() const { return m_modelRoiVisible; }
bool ShapeModelDebugViewModel::modelRoiEditable() const { return m_modelRoiEditable; }
QRectF ShapeModelDebugViewModel::searchRoi() const { return m_searchRoi; }
bool ShapeModelDebugViewModel::searchRoiEnabled() const { return m_searchRoiEnabled; }
bool ShapeModelDebugViewModel::coarseMatchingRunning() const { return m_coarseMatchingRunning; }
bool ShapeModelDebugViewModel::coarseMatchFinished() const { return m_coarseMatchFinished; }
QString ShapeModelDebugViewModel::statisticsText() const { return m_statisticsText; }
QString ShapeModelDebugViewModel::status() const { return m_status; }

QString ShapeModelDebugViewModel::groundTruthSummary() const { return m_groundTruthSummary; }

void ShapeModelDebugViewModel::bindDisplay(QObject* displayObject)
{
    m_display = qobject_cast<VisionDisplay::VisionDisplayItem*>(displayObject);
    if (m_display) {
        m_display->setShowCrosshair(true);
        m_display->setModelRoi(m_modelRoi);
        m_display->setModelRoiVisible(m_modelRoiVisible);
        m_display->setModelRoiEditable(m_modelRoiEditable);
        connect(m_display.data(), &VisionDisplay::VisionDisplayItem::roiChanged, this, [this](const QString& id) {
            if (id == QString::fromLatin1(searchRoiId)) {
                syncSearchRoiFromDisplay();
            }
        });
        connect(m_display.data(), &VisionDisplay::VisionDisplayItem::roiCreated, this, [this](const QString& id) {
            if (id == QString::fromLatin1(searchRoiId)) {
                syncSearchRoiFromDisplay();
            }
        });
        if (m_searchRoiEnabled) {
            createOrUpdateSearchRoiOnDisplay();
        }
        m_display->clearOverlayData(QStringLiteral("ShapeModelOverlay"));
        m_display->clearOverlayData(QStringLiteral("CoarseMatchOverlay"));
        if (!m_templateImage.isNull()) {
            updateDisplayedLevelImage();
            refreshOverlay();
        }
        setStatus(QStringLiteral("Display ready."));
    } else {
        setStatus(QStringLiteral("Display bind failed."));
    }
}

bool ShapeModelDebugViewModel::loadTemplateImage(const QUrl& url)
{
    QImageReader reader(localPath(url));
    reader.setAutoTransform(true);
    QImage image = reader.read();
    if (image.isNull()) {
        setStatus(QStringLiteral("Load failed: %1").arg(reader.errorString()));
        return false;
    }
    if (image.format() != QImage::Format_Grayscale8) {
        image = image.convertToFormat(QImage::Format_Grayscale8);
    }

    m_templateImage = image;
    const double roiWidth = std::max(32.0, image.width() * 0.6);
    const double roiHeight = std::max(32.0, image.height() * 0.6);
    m_modelRoi = QRectF((image.width() - roiWidth) * 0.5,
                        (image.height() - roiHeight) * 0.5,
                        roiWidth,
                        roiHeight);
    m_searchRoi = defaultSearchRoi();
    m_searchRoiEnabled = false;
    m_modelRoiVisible = true;
    m_modelRoiEditable = true;
    m_model = VisionTools::Matching::ShapeTemplateModel();
    m_currentLevel = 0;
    m_lastCoarseSummary.clear();
    m_groundTruthPath.clear();
    m_groundTruthSummary.clear();
    m_visionProTrainingOrigin = QPointF();
    m_hasVisionProTrainingOrigin = false;
    setCoarseMatchFinished(false);
    emit modelRoiChanged();
    emit searchRoiChanged();
    emit searchRoiEnabledChanged();
    emit modelRoiVisibleChanged();
    emit modelRoiEditableChanged();
    emit currentLevelChanged();
    emit levelCountChanged();
    emit groundTruthChanged();
    if (m_display) {
        m_display->setKeepViewTransformOnNewImage(false);
        m_display->setAutoFitOnNewImage(true);
        m_display->setImage(m_templateImage);
        m_display->fitToWindow();
        m_display->setModelRoi(m_modelRoi);
        m_display->setModelRoiVisible(m_modelRoiVisible);
        m_display->setModelRoiEditable(m_modelRoiEditable);
        m_display->clearOverlayData(QStringLiteral("ShapeModelOverlay"));
        m_display->clearOverlayData(QStringLiteral("CoarseMatchOverlay"));
    }
    updateStatistics();
    setStatus(QStringLiteral("Loaded %1 (%2x%3)")
                  .arg(QFileInfo(localPath(url)).fileName())
                  .arg(m_templateImage.width())
                  .arg(m_templateImage.height()));
    return true;
}

bool ShapeModelDebugViewModel::buildModel()
{
    return buildModelFromRoi();
}

bool ShapeModelDebugViewModel::buildModelFromRoi()
{
    if (m_templateImage.isNull()) {
        setStatus(QStringLiteral("Load a template image first."));
        return false;
    }
    const QRect imageRect(0, 0, m_templateImage.width(), m_templateImage.height());
    const QRect roi = m_modelRoi.toAlignedRect().intersected(imageRect);
    if (roi.width() < 32 || roi.height() < 32) {
        setStatus(QStringLiteral("Model ROI is too small. Minimum is 32 x 32."));
        return false;
    }

    VisionTools::Matching::ShapeModelParams params;
    params.pyramidLevels = 4;
    params.buildDebugImage = true;
    params.debugOutputDir = QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("shape_model_debug"));

    VisionTools::Matching::ShapeModelBuilder builder;
    m_model = m_hasVisionProTrainingOrigin
        ? builder.build(m_templateImage, roi, params, m_visionProTrainingOrigin)
        : builder.build(m_templateImage, roi, params);
    m_lastCoarseSummary.clear();
    setCoarseMatchFinished(false);
    m_currentLevel = std::clamp(m_currentLevel, 0, std::max(0, levelCount() - 1));
    emit levelCountChanged();
    emit currentLevelChanged();
    updateDisplayedLevelImage();
    refreshOverlay();
    if (m_model.isValid()) {
        setStatus(QStringLiteral("Shape model built. points=%1 levels=%2").arg(m_model.totalPointCount()).arg(levelCount()));
        return true;
    }
    if (hasCoarseTemplateSource(m_model)) {
        setStatus(QStringLiteral("No stable sampled points. Coarse match will use edge candidates fallback.\n%1").arg(m_model.message));
        return true;
    }
    setStatus(m_model.message);
    return false;
}

void ShapeModelDebugViewModel::setCurrentLevel(int level)
{
    const int bounded = std::clamp(level, 0, std::max(0, levelCount() - 1));
    if (m_currentLevel == bounded) {
        return;
    }
    m_currentLevel = bounded;
    emit currentLevelChanged();
    updateDisplayedLevelImage();
    refreshOverlay();
}

void ShapeModelDebugViewModel::refreshOverlay()
{
    if (!m_display || levelCount() <= 0) {
        updateStatistics();
        return;
    }

    const VisionDisplay::VisionDisplayOverlayData overlay =
        ShapeModelOverlayBuilder::build(m_model, m_currentLevel, m_options);
    m_display->setOverlayData(QStringLiteral("ShapeModelOverlay"), overlay);
    updateStatistics();
}

void ShapeModelDebugViewModel::runCoarseMatch()
{
    if (m_coarseMatchingRunning) {
        return;
    }
    if (!m_display) {
        setStatus(QStringLiteral("Display is not ready."));
        return;
    }
    if (m_templateImage.isNull()) {
        setStatus(QStringLiteral("Load an image first."));
        return;
    }
    if (!hasCoarseTemplateSource(m_model)) {
        setStatus(QStringLiteral("Build model from ROI first. Current ROI has no usable edge candidates."));
        return;
    }

    const QImage image = m_templateImage.copy();
    const VisionTools::Matching::ShapeTemplateModel model = m_model;
    const QString groundTruthPath = m_groundTruthPath;
    const bool useSearchRoi = m_searchRoiEnabled && !m_searchRoi.isEmpty();
    const QRect searchRect = useSearchRoi
        ? boundedRoiRect(m_searchRoi, m_templateImage.size(), 32)
        : QRect();
    if (useSearchRoi && searchRect.isEmpty()) {
        setStatus(QStringLiteral("Search ROI is too small. Minimum is 32 x 32."));
        return;
    }
    m_lastCoarseSummary.clear();
    setCoarseMatchFinished(false);
    setCoarseMatchingRunning(true);
    setStatus(QStringLiteral("Coarse matching running..."));
    m_display->clearOverlayData(QStringLiteral("CoarseMatchOverlay"));

    m_coarseWatcher.setFuture(QtConcurrent::run([image, model, searchRect, groundTruthPath]() {
        CoarseMatchUiResult result;
        const ShapeMatch::ShapeTemplateModel coarseModel = toCoarseTemplateModel(model);
        if (coarseModel.empty()) {
            result.message = QStringLiteral("Coarse match failed: model has no template points.");
            return result;
        }
        const ShapeMatch::EdgeImageData edgeData = edgeDataFromImage(image, searchRect);
        if (edgeData.empty()) {
            result.message = QStringLiteral("Coarse match failed: no edge data.");
            return result;
        }

        ShapeMatch::CoarseMatchConfig coarseConfig;
        coarseConfig.pyramidLevels = 4;
        coarseConfig.topKPerLevel = 500;
        coarseConfig.finalTopK = 160;
        coarseConfig.beamWidth = 420;
        coarseConfig.coarseAngleStepDeg = 10.0;
        coarseConfig.fineAngleStepDeg = 2.0;
        coarseConfig.coarseTranslationStepPx = 8;
        coarseConfig.fineTranslationStepPx = 1;
        coarseConfig.localRefineRadiusPx = 9;
        coarseConfig.localRefineAngleRadiusDeg = 7;
        coarseConfig.maxTemplatePointsPerLevel = 120;
        coarseConfig.minTemplatePointsForScore = 20;
        coarseConfig.fastDistanceSigma = 2.0;
        coarseConfig.maxCandidatesEvaluatedPerLevel = 240000;
        coarseConfig.nmsTranslationThresholdPx = 5.0;
        coarseConfig.nmsAngleThresholdDeg = 4.0;
        coarseConfig.maxVotingCandidatesToScore = 1200;
        coarseConfig.maxCandidatesPerLevel = 220000;
        coarseConfig.maxCandidatesLevel0 = 120000;
        coarseConfig.maxCandidatesLocalRefine = 120000;
        coarseConfig.maxChildrenPerParentLevel0 = 700;
        coarseConfig.maxChildrenPerParentCoarse = 420;
        coarseConfig.maxBeamForLocalRefineLevel0 = 180;
        coarseConfig.maxBeamForLocalRefineCoarse = 420;
        coarseConfig.maxParentsForLevel0Refine = 180;
        coarseConfig.maxParentsPerGridCell = 10;
        coarseConfig.parentDiversityGridRows = 16;
        coarseConfig.parentDiversityGridCols = 16;
        coarseConfig.diversityGridRows = 16;
        coarseConfig.diversityGridCols = 16;
        coarseConfig.topKPerGridCell = 10;
        coarseConfig.globalTopKAfterDiversity = 500;
        if (!searchRect.isEmpty()) {
            coarseConfig.enableSearchRoi = true;
            coarseConfig.searchRoiX = searchRect.x();
            coarseConfig.searchRoiY = searchRect.y();
            coarseConfig.searchRoiWidth = searchRect.width();
            coarseConfig.searchRoiHeight = searchRect.height();
        }

        ShapeMatch::ShapeMatchEvalConfig evalConfig;
        evalConfig.topK = coarseConfig.finalTopK;
        evalConfig.maxFinalRankerInputCandidates = coarseConfig.finalTopK;
        evalConfig.targetFinalRankerInputCandidates = coarseConfig.finalTopK;
        evalConfig.searchRadiusPx = 5.0;
        evalConfig.minValidPoints = 20;
        evalConfig.minCoverageRatio = 0.65;
        evalConfig.minInlierRatio = 0.45;
        evalConfig.maxMedianErrorPx = 3.0;
        evalConfig.maxP90ErrorPx = 8.0;

        std::vector<ShapeMatch::GroundTruthInstance> gt;
        QString gtLabel;
        if (!groundTruthPath.isEmpty()) {
            ShapeMatch::VisionProGroundTruthReader reader;
            const ShapeMatch::VisionProGroundTruthReadResult gtRead = reader.read(groundTruthPath.toStdString());
            if (gtRead.ok()) {
                gt = gtRead.instances;
                gtLabel = QStringLiteral("VisionProGT=%1").arg(static_cast<int>(gt.size()));
            } else {
                result.message = QStringLiteral("VisionPro GT load failed: %1").arg(QString::fromStdString(gtRead.error));
                return result;
            }
        } else {
            const ShapeMatch::GroundTruthInstance expected{"model_roi_origin",
                ShapeMatch::MatchPose::fromDeg(model.originImage.x(), model.originImage.y(), 0.0, 1.0)};
            gt.push_back(expected);
            gtLabel = QStringLiteral("pseudoGT=model_roi_origin");
        }

        ShapeMatch::CoarseShapeMatcher matcher(coarseConfig, evalConfig);
        result.report = matcher.match(coarseModel, edgeData, &gt);
        if (result.report.finalRankedCandidates.empty()) {
            result.message = QStringLiteral("Coarse match produced no candidates: %1")
                                 .arg(QString::fromStdString(result.report.failureReason));
            return result;
        }

        ShapeMatch::CoarseMatchDebugOverlay overlayBuilder;
        const ShapeMatch::CoarseMatchReport overlayReport =
            spatiallyFilteredReport(result.report, 160, 0.30, 0.15, 0.05, 10.0, 25.0, 5.0, 2.0);
        result.overlay = overlayBuilder.buildOverlayData(overlayReport, coarseModel, 80);
        const ShapeMatch::ScoredCandidate& top = result.report.finalRankedCandidates.front();
        if (overlayReport.finalRankedCandidates.empty()) {
            result.ok = false;
            result.message = QStringLiteral("Coarse match done, but no displayable accepted ROI. %1 top1 x=%2 y=%3 theta=%4 score=%5 cov=%6 inlier=%7; reports=data/shape_match/reports")
                                 .arg(gtLabel)
                                 .arg(top.pose.x, 0, 'f', 1)
                                 .arg(top.pose.y, 0, 'f', 1)
                                 .arg(top.pose.thetaDeg(), 0, 'f', 2)
                                 .arg(top.score.finalScore, 0, 'f', 3)
                                 .arg(top.score.coverageRatio, 0, 'f', 3)
                                 .arg(top.score.inlierRatio, 0, 'f', 3);
            return result;
        }
        result.ok = true;
        const ShapeMatch::ScoredCandidate& drawnTop = overlayReport.finalRankedCandidates.front();
        const QString roiText = searchRect.isEmpty()
            ? QStringLiteral("full image")
            : QStringLiteral("searchRoi=(%1,%2,%3,%4)")
                  .arg(searchRect.x())
                  .arg(searchRect.y())
                  .arg(searchRect.width())
                  .arg(searchRect.height());
        result.message = QStringLiteral("Coarse match done. %1 %2 drawn1 x=%3 y=%4 theta=%5 score=%6 cov=%7 inlier=%8 drawn=%9; reports=data/shape_match/reports")
                             .arg(roiText)
                             .arg(gtLabel)
                             .arg(drawnTop.pose.x, 0, 'f', 1)
                             .arg(drawnTop.pose.y, 0, 'f', 1)
                             .arg(drawnTop.pose.thetaDeg(), 0, 'f', 2)
                             .arg(drawnTop.score.finalScore, 0, 'f', 3)
                             .arg(drawnTop.score.coverageRatio, 0, 'f', 3)
                             .arg(drawnTop.score.inlierRatio, 0, 'f', 3)
                             .arg(static_cast<int>(overlayReport.finalRankedCandidates.size()));
        return result;
    }));
}

bool ShapeModelDebugViewModel::saveDebugImages()
{
    if (levelCount() <= 0) {
        setStatus(QStringLiteral("Build model first."));
        return false;
    }

    const QString path = QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("shape_model_debug"));
    setStatus(QStringLiteral("Debug images saved to %1").arg(QDir::toNativeSeparators(path)));
    return true;
}

bool ShapeModelDebugViewModel::loadVisionProGroundTruth(const QUrl& url)
{
    const QString path = localPath(url);
    ShapeMatch::VisionProGroundTruthReader reader;
    const ShapeMatch::VisionProGroundTruthReadResult result = reader.read(path.toStdString());
    if (!result.ok()) {
        setStatus(QStringLiteral("VisionPro GT load failed: %1").arg(QString::fromStdString(result.error)));
        return false;
    }

    const QFileInfo gtInfo(path);
    QString imagePath = QString::fromStdString(result.imageName);
    if (imagePath.isEmpty()) {
        setStatus(QStringLiteral("VisionPro GT load failed: missing image field."));
        return false;
    }
    QFileInfo imageInfo(imagePath);
    if (imageInfo.isRelative()) {
        imagePath = gtInfo.dir().filePath(imagePath);
        imageInfo = QFileInfo(imagePath);
    }

    QImageReader imageReader(imagePath);
    imageReader.setAutoTransform(true);
    QImage image = imageReader.read();
    if (image.isNull()) {
        setStatus(QStringLiteral("VisionPro GT image load failed: %1 (%2)")
                      .arg(imageReader.errorString(), QDir::toNativeSeparators(imagePath)));
        return false;
    }
    if (image.format() != QImage::Format_Grayscale8) {
        image = image.convertToFormat(QImage::Format_Grayscale8);
    }

    m_groundTruthPath = path;
    m_templateImage = image;
    if (result.training.hasTrainingRoi) {
        m_modelRoi = QRectF(result.training.roiX,
                            result.training.roiY,
                            result.training.roiWidth,
                            result.training.roiHeight).normalized();
    } else {
        const double roiWidth = std::max(32.0, image.width() * 0.6);
        const double roiHeight = std::max(32.0, image.height() * 0.6);
        m_modelRoi = QRectF((image.width() - roiWidth) * 0.5,
                            (image.height() - roiHeight) * 0.5,
                            roiWidth,
                            roiHeight);
    }
    if (result.training.hasOrigin) {
        m_visionProTrainingOrigin = QPointF(result.training.originX, result.training.originY);
        m_hasVisionProTrainingOrigin = true;
    } else {
        m_visionProTrainingOrigin = QPointF();
        m_hasVisionProTrainingOrigin = false;
    }
    m_searchRoi = defaultSearchRoi();
    m_searchRoiEnabled = false;
    m_modelRoiVisible = true;
    m_modelRoiEditable = true;
    m_model = VisionTools::Matching::ShapeTemplateModel();
    m_currentLevel = 0;
    m_lastCoarseSummary.clear();
    setCoarseMatchFinished(false);
    m_groundTruthSummary = QStringLiteral("VisionPro GT: %1 targets (%2)")
                               .arg(static_cast<int>(result.instances.size()))
                               .arg(gtInfo.fileName());
    emit modelRoiChanged();
    emit searchRoiChanged();
    emit searchRoiEnabledChanged();
    emit modelRoiVisibleChanged();
    emit modelRoiEditableChanged();
    emit currentLevelChanged();
    emit levelCountChanged();
    emit groundTruthChanged();
    if (m_display) {
        m_display->setKeepViewTransformOnNewImage(false);
        m_display->setAutoFitOnNewImage(true);
        m_display->setImage(m_templateImage);
        m_display->fitToWindow();
        m_display->setModelRoi(m_modelRoi);
        m_display->setModelRoiVisible(m_modelRoiVisible);
        m_display->setModelRoiEditable(m_modelRoiEditable);
        m_display->clearOverlayData(QStringLiteral("ShapeModelOverlay"));
        m_display->clearOverlayData(QStringLiteral("CoarseMatchOverlay"));
    }
    updateStatistics();
    setStatus(QStringLiteral("%1; loaded image %2 (%3x%4); model ROI from VisionPro training.")
                  .arg(m_groundTruthSummary)
                  .arg(imageInfo.fileName())
                  .arg(m_templateImage.width())
                  .arg(m_templateImage.height()));
    return true;
}

void ShapeModelDebugViewModel::clearGroundTruth()
{
    m_groundTruthPath.clear();
    m_groundTruthSummary.clear();
    m_visionProTrainingOrigin = QPointF();
    m_hasVisionProTrainingOrigin = false;
    emit groundTruthChanged();
    setStatus(QStringLiteral("VisionPro GT cleared."));
}

void ShapeModelDebugViewModel::resetModelRoi()
{
    if (m_templateImage.isNull()) {
        return;
    }
    const double roiWidth = std::max(32.0, m_templateImage.width() * 0.6);
    const double roiHeight = std::max(32.0, m_templateImage.height() * 0.6);
    setModelRoi(QRectF((m_templateImage.width() - roiWidth) * 0.5,
                       (m_templateImage.height() - roiHeight) * 0.5,
                       roiWidth,
                       roiHeight));
    setModelRoiVisible(true);
}

void ShapeModelDebugViewModel::enableSearchRoi()
{
    if (m_templateImage.isNull()) {
        setStatus(QStringLiteral("Load an image first."));
        return;
    }
    if (m_searchRoi.isEmpty()) {
        m_searchRoi = defaultSearchRoi();
        emit searchRoiChanged();
    }
    setSearchRoiEnabled(true);
    setModelRoiEditable(false);
    createOrUpdateSearchRoiOnDisplay();
    setStatus(QStringLiteral("Search ROI enabled. Drag or resize the yellow rectangle, then run coarse match."));
}

void ShapeModelDebugViewModel::resetSearchRoi()
{
    if (m_templateImage.isNull()) {
        return;
    }
    setSearchRoi(defaultSearchRoi());
    setSearchRoiEnabled(true);
    createOrUpdateSearchRoiOnDisplay();
}

void ShapeModelDebugViewModel::setShowAllEdges(bool enabled) { if (setOption(&m_options.showAllEdges, enabled)) refreshOverlay(); }
void ShapeModelDebugViewModel::setShowStablePoints(bool enabled) { if (setOption(&m_options.showStablePoints, enabled)) refreshOverlay(); }
void ShapeModelDebugViewModel::setShowChains(bool enabled) { if (setOption(&m_options.showChains, enabled)) refreshOverlay(); }
void ShapeModelDebugViewModel::setShowSampledPoints(bool enabled) { if (setOption(&m_options.showSampledPoints, enabled)) refreshOverlay(); }
void ShapeModelDebugViewModel::setShowNormals(bool enabled) { if (setOption(&m_options.showNormals, enabled)) refreshOverlay(); }
void ShapeModelDebugViewModel::setShowTangents(bool enabled) { if (setOption(&m_options.showTangents, enabled)) refreshOverlay(); }
void ShapeModelDebugViewModel::setShowCurvature(bool enabled) { if (setOption(&m_options.showCurvature, enabled)) refreshOverlay(); }
void ShapeModelDebugViewModel::setShowRejectedPoints(bool enabled) { if (setOption(&m_options.showRejectedPoints, enabled)) refreshOverlay(); }
void ShapeModelDebugViewModel::setShowChainId(bool enabled) { if (setOption(&m_options.showChainId, enabled)) refreshOverlay(); }
void ShapeModelDebugViewModel::setShowPointId(bool enabled) { if (setOption(&m_options.showPointId, enabled)) refreshOverlay(); }

void ShapeModelDebugViewModel::setNormalStep(int step)
{
    const int next = std::max(1, step);
    if (m_options.normalStep == next) {
        return;
    }
    m_options.normalStep = next;
    emit overlayOptionsChanged();
    refreshOverlay();
}

void ShapeModelDebugViewModel::setNormalLength(double length)
{
    const double next = std::max(1.0, length);
    if (qFuzzyCompare(m_options.normalLength, next)) {
        return;
    }
    m_options.normalLength = next;
    emit overlayOptionsChanged();
    refreshOverlay();
}

void ShapeModelDebugViewModel::setTangentStep(int step)
{
    const int next = std::max(1, step);
    if (m_options.tangentStep == next) {
        return;
    }
    m_options.tangentStep = next;
    emit overlayOptionsChanged();
    refreshOverlay();
}

void ShapeModelDebugViewModel::setTangentLength(double length)
{
    const double next = std::max(1.0, length);
    if (qFuzzyCompare(m_options.tangentLength, next)) {
        return;
    }
    m_options.tangentLength = next;
    emit overlayOptionsChanged();
    refreshOverlay();
}

void ShapeModelDebugViewModel::setModelRoi(const QRectF& roi)
{
    QRectF next = roi.normalized();
    if (!m_templateImage.isNull()) {
        const QRectF imageRect(0.0, 0.0, m_templateImage.width(), m_templateImage.height());
        next = next.intersected(imageRect).normalized();
        if (next.width() < 32.0) {
            next.setWidth(std::min(32.0, imageRect.width()));
        }
        if (next.height() < 32.0) {
            next.setHeight(std::min(32.0, imageRect.height()));
        }
        if (next.right() > imageRect.right()) {
            next.moveRight(imageRect.right());
        }
        if (next.bottom() > imageRect.bottom()) {
            next.moveBottom(imageRect.bottom());
        }
        if (next.left() < imageRect.left()) {
            next.moveLeft(imageRect.left());
        }
        if (next.top() < imageRect.top()) {
            next.moveTop(imageRect.top());
        }
    }

    if (m_modelRoi == next) {
        return;
    }
    m_modelRoi = next;
    if (m_display && m_display->modelRoi() != m_modelRoi) {
        m_display->setModelRoi(m_modelRoi);
    }
    emit modelRoiChanged();
    updateStatistics();
}

void ShapeModelDebugViewModel::setModelRoiVisible(bool visible)
{
    if (m_modelRoiVisible == visible) {
        return;
    }
    m_modelRoiVisible = visible;
    if (m_display) {
        m_display->setModelRoiVisible(visible);
    }
    emit modelRoiVisibleChanged();
}

void ShapeModelDebugViewModel::setModelRoiEditable(bool editable)
{
    if (m_modelRoiEditable == editable) {
        return;
    }
    m_modelRoiEditable = editable;
    if (m_display) {
        m_display->setModelRoiEditable(editable);
    }
    emit modelRoiEditableChanged();
}

void ShapeModelDebugViewModel::setSearchRoi(const QRectF& roi)
{
    const QRectF next = boundedImageRect(roi, 32.0);
    if (m_searchRoi == next) {
        return;
    }
    m_searchRoi = next;
    if (m_searchRoiEnabled) {
        createOrUpdateSearchRoiOnDisplay();
    }
    emit searchRoiChanged();
    updateStatistics();
}

void ShapeModelDebugViewModel::setSearchRoiEnabled(bool enabled)
{
    if (m_searchRoiEnabled == enabled) {
        return;
    }
    m_searchRoiEnabled = enabled;
    if (m_searchRoiEnabled && m_searchRoi.isEmpty()) {
        m_searchRoi = defaultSearchRoi();
        emit searchRoiChanged();
    }
    if (m_searchRoiEnabled) {
        createOrUpdateSearchRoiOnDisplay();
    }
    emit searchRoiEnabledChanged();
    updateStatistics();
}

void ShapeModelDebugViewModel::setCoarseMatchingRunning(bool running)
{
    if (m_coarseMatchingRunning == running) {
        return;
    }
    m_coarseMatchingRunning = running;
    emit coarseMatchingRunningChanged();
}

void ShapeModelDebugViewModel::setCoarseMatchFinished(bool finished)
{
    if (m_coarseMatchFinished == finished) {
        return;
    }
    m_coarseMatchFinished = finished;
    emit coarseMatchFinishedChanged();
}

void ShapeModelDebugViewModel::setStatus(const QString& status)
{
    if (m_status == status) {
        return;
    }
    m_status = status;
    emit statusChanged();
}

void ShapeModelDebugViewModel::updateStatistics()
{
    QString text;
    if (levelCount() <= 0) {
        text = QStringLiteral("No model built.");
    } else {
        const auto& level = m_model.levels[static_cast<size_t>(m_currentLevel)];
        text = QStringLiteral("Level %1/%2\nsize=%3x%4 scale=%5\nchains=%6\nallEdges=%7\nstable=%8\nsampled=%9\nrejected=%10\n\n%11")
                   .arg(m_currentLevel)
                   .arg(levelCount() - 1)
                   .arg(level.width)
                   .arg(level.height)
                   .arg(level.scale, 0, 'f', 3)
                   .arg(static_cast<int>(level.chains.size()))
                   .arg(static_cast<int>(level.allEdgePoints.size()))
                   .arg(static_cast<int>(level.stablePoints.size()))
                   .arg(static_cast<int>(level.points.size()))
                   .arg(static_cast<int>(level.rejectedPoints.size()))
                   .arg(level.message);
    }
    if (!m_modelRoi.isEmpty()) {
        text += QStringLiteral("\n\nROI x=%1 y=%2 w=%3 h=%4")
                    .arg(m_modelRoi.x(), 0, 'f', 1)
                    .arg(m_modelRoi.y(), 0, 'f', 1)
                    .arg(m_modelRoi.width(), 0, 'f', 1)
                    .arg(m_modelRoi.height(), 0, 'f', 1);
    }
    if (m_searchRoiEnabled && !m_searchRoi.isEmpty()) {
        text += QStringLiteral("\nSearch ROI x=%1 y=%2 w=%3 h=%4")
                    .arg(m_searchRoi.x(), 0, 'f', 1)
                    .arg(m_searchRoi.y(), 0, 'f', 1)
                    .arg(m_searchRoi.width(), 0, 'f', 1)
                    .arg(m_searchRoi.height(), 0, 'f', 1);
    }
    if (!m_lastCoarseSummary.isEmpty()) {
        text += QStringLiteral("\n\nCoarse Match\n%1").arg(m_lastCoarseSummary);
    }

    if (m_statisticsText == text) {
        return;
    }
    m_statisticsText = text;
    emit statisticsTextChanged();
}

void ShapeModelDebugViewModel::updateDisplayedLevelImage()
{
    if (!m_display) {
        return;
    }
    if (levelCount() > 0) {
        m_display->setKeepViewTransformOnNewImage(false);
        m_display->setAutoFitOnNewImage(true);
        m_display->setImage(m_templateImage);
        m_display->fitToWindow();
    } else if (!m_templateImage.isNull()) {
        m_display->setImage(m_templateImage);
        m_display->fitToWindow();
    }
}

bool ShapeModelDebugViewModel::setOption(bool* target, bool value)
{
    if (*target == value) {
        return false;
    }
    *target = value;
    emit overlayOptionsChanged();
    return true;
}

QRectF ShapeModelDebugViewModel::defaultSearchRoi() const
{
    if (m_templateImage.isNull()) {
        return {};
    }
    const double roiWidth = std::max(32.0, m_templateImage.width() * 0.8);
    const double roiHeight = std::max(32.0, m_templateImage.height() * 0.8);
    return QRectF((m_templateImage.width() - roiWidth) * 0.5,
                  (m_templateImage.height() - roiHeight) * 0.5,
                  roiWidth,
                  roiHeight);
}

QRectF ShapeModelDebugViewModel::boundedImageRect(const QRectF& rect, double minSize) const
{
    QRectF next = rect.normalized();
    if (m_templateImage.isNull()) {
        return next;
    }
    const QRectF imageRect(0.0, 0.0, m_templateImage.width(), m_templateImage.height());
    next = next.intersected(imageRect).normalized();
    if (next.width() < minSize) {
        next.setWidth(std::min(minSize, imageRect.width()));
    }
    if (next.height() < minSize) {
        next.setHeight(std::min(minSize, imageRect.height()));
    }
    if (next.right() > imageRect.right()) {
        next.moveRight(imageRect.right());
    }
    if (next.bottom() > imageRect.bottom()) {
        next.moveBottom(imageRect.bottom());
    }
    if (next.left() < imageRect.left()) {
        next.moveLeft(imageRect.left());
    }
    if (next.top() < imageRect.top()) {
        next.moveTop(imageRect.top());
    }
    return next;
}

void ShapeModelDebugViewModel::createOrUpdateSearchRoiOnDisplay()
{
    if (!m_display || !m_searchRoiEnabled || m_searchRoi.isEmpty()) {
        return;
    }
    m_display->createRectRoi(QString::fromLatin1(searchRoiId),
                             m_searchRoi.x(),
                             m_searchRoi.y(),
                             m_searchRoi.width(),
                             m_searchRoi.height());
}

void ShapeModelDebugViewModel::syncSearchRoiFromDisplay()
{
    if (!m_display) {
        return;
    }
    const QVariantMap geometry = m_display->roiGeometry(QString::fromLatin1(searchRoiId));
    if (!geometry.value(QStringLiteral("valid")).toBool()
        || geometry.value(QStringLiteral("type")).toString() != QStringLiteral("rect")) {
        return;
    }
    const QRectF next = boundedImageRect(QRectF(geometry.value(QStringLiteral("x")).toDouble(),
                                               geometry.value(QStringLiteral("y")).toDouble(),
                                               geometry.value(QStringLiteral("width")).toDouble(),
                                               geometry.value(QStringLiteral("height")).toDouble()),
                                         32.0);
    if (m_searchRoi == next) {
        return;
    }
    m_searchRoi = next;
    emit searchRoiChanged();
    updateStatistics();
}
