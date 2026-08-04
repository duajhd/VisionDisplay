#pragma once

#include "ShapeModelOverlayBuilder.h"
#include "shape_match/coarse/CoarseMatchReport.h"
#include "shape_match/overlay/ShapeMatchOverlayData.h"
#include "VisionTools/Matching/ShapeTemplateModel.h"

#include <QImage>
#include <QFutureWatcher>
#include <QObject>
#include <QPointer>
#include <QPointF>
#include <QString>
#include <QUrl>
#include <QRectF>

namespace VisionDisplay {
class VisionDisplayItem;
}

class ShapeModelDebugViewModel : public QObject
{
    Q_OBJECT
    Q_PROPERTY(int currentLevel READ currentLevel WRITE setCurrentLevel NOTIFY currentLevelChanged)
    Q_PROPERTY(int levelCount READ levelCount NOTIFY levelCountChanged)
    Q_PROPERTY(bool showAllEdges READ showAllEdges WRITE setShowAllEdges NOTIFY overlayOptionsChanged)
    Q_PROPERTY(bool showStablePoints READ showStablePoints WRITE setShowStablePoints NOTIFY overlayOptionsChanged)
    Q_PROPERTY(bool showChains READ showChains WRITE setShowChains NOTIFY overlayOptionsChanged)
    Q_PROPERTY(bool showSampledPoints READ showSampledPoints WRITE setShowSampledPoints NOTIFY overlayOptionsChanged)
    Q_PROPERTY(bool showNormals READ showNormals WRITE setShowNormals NOTIFY overlayOptionsChanged)
    Q_PROPERTY(bool showTangents READ showTangents WRITE setShowTangents NOTIFY overlayOptionsChanged)
    Q_PROPERTY(bool showCurvature READ showCurvature WRITE setShowCurvature NOTIFY overlayOptionsChanged)
    Q_PROPERTY(bool showRejectedPoints READ showRejectedPoints WRITE setShowRejectedPoints NOTIFY overlayOptionsChanged)
    Q_PROPERTY(bool showChainId READ showChainId WRITE setShowChainId NOTIFY overlayOptionsChanged)
    Q_PROPERTY(bool showPointId READ showPointId WRITE setShowPointId NOTIFY overlayOptionsChanged)
    Q_PROPERTY(int normalStep READ normalStep WRITE setNormalStep NOTIFY overlayOptionsChanged)
    Q_PROPERTY(double normalLength READ normalLength WRITE setNormalLength NOTIFY overlayOptionsChanged)
    Q_PROPERTY(int tangentStep READ tangentStep WRITE setTangentStep NOTIFY overlayOptionsChanged)
    Q_PROPERTY(double tangentLength READ tangentLength WRITE setTangentLength NOTIFY overlayOptionsChanged)
    Q_PROPERTY(QRectF modelRoi READ modelRoi WRITE setModelRoi NOTIFY modelRoiChanged)
    Q_PROPERTY(bool modelRoiVisible READ modelRoiVisible WRITE setModelRoiVisible NOTIFY modelRoiVisibleChanged)
    Q_PROPERTY(bool modelRoiEditable READ modelRoiEditable WRITE setModelRoiEditable NOTIFY modelRoiEditableChanged)
    Q_PROPERTY(QRectF searchRoi READ searchRoi WRITE setSearchRoi NOTIFY searchRoiChanged)
    Q_PROPERTY(bool searchRoiEnabled READ searchRoiEnabled WRITE setSearchRoiEnabled NOTIFY searchRoiEnabledChanged)
    Q_PROPERTY(bool coarseMatchingRunning READ coarseMatchingRunning NOTIFY coarseMatchingRunningChanged)
    Q_PROPERTY(bool coarseMatchFinished READ coarseMatchFinished NOTIFY coarseMatchFinishedChanged)
    Q_PROPERTY(QString groundTruthSummary READ groundTruthSummary NOTIFY groundTruthChanged)
    Q_PROPERTY(QString statisticsText READ statisticsText NOTIFY statisticsTextChanged)
    Q_PROPERTY(QString status READ status NOTIFY statusChanged)

public:
    explicit ShapeModelDebugViewModel(QObject* parent = nullptr);

    int currentLevel() const;
    int levelCount() const;
    bool showAllEdges() const;
    bool showStablePoints() const;
    bool showChains() const;
    bool showSampledPoints() const;
    bool showNormals() const;
    bool showTangents() const;
    bool showCurvature() const;
    bool showRejectedPoints() const;
    bool showChainId() const;
    bool showPointId() const;
    int normalStep() const;
    double normalLength() const;
    int tangentStep() const;
    double tangentLength() const;
    QRectF modelRoi() const;
    bool modelRoiVisible() const;
    bool modelRoiEditable() const;
    QRectF searchRoi() const;
    bool searchRoiEnabled() const;
    bool coarseMatchingRunning() const;
    bool coarseMatchFinished() const;
    QString groundTruthSummary() const;
    QString statisticsText() const;
    QString status() const;

    Q_INVOKABLE void bindDisplay(QObject* displayObject);
    Q_INVOKABLE bool loadTemplateImage(const QUrl& url);
    Q_INVOKABLE bool buildModel();
    Q_INVOKABLE bool buildModelFromRoi();
    Q_INVOKABLE void setCurrentLevel(int level);
    Q_INVOKABLE void refreshOverlay();
    Q_INVOKABLE bool saveDebugImages();
    Q_INVOKABLE bool loadVisionProGroundTruth(const QUrl& url);
    Q_INVOKABLE void clearGroundTruth();
    Q_INVOKABLE void resetModelRoi();
    Q_INVOKABLE void enableSearchRoi();
    Q_INVOKABLE void resetSearchRoi();
    Q_INVOKABLE void runCoarseMatch();
    Q_INVOKABLE void runPipelineV2Match();
    Q_INVOKABLE void runShapeMatchV3();

    void setShowAllEdges(bool enabled);
    void setShowStablePoints(bool enabled);
    void setShowChains(bool enabled);
    void setShowSampledPoints(bool enabled);
    void setShowNormals(bool enabled);
    void setShowTangents(bool enabled);
    void setShowCurvature(bool enabled);
    void setShowRejectedPoints(bool enabled);
    void setShowChainId(bool enabled);
    void setShowPointId(bool enabled);
    void setNormalStep(int step);
    void setNormalLength(double length);
    void setTangentStep(int step);
    void setTangentLength(double length);
    void setModelRoi(const QRectF& roi);
    void setModelRoiVisible(bool visible);
    void setModelRoiEditable(bool editable);
    void setSearchRoi(const QRectF& roi);
    void setSearchRoiEnabled(bool enabled);

signals:
    void currentLevelChanged();
    void levelCountChanged();
    void overlayOptionsChanged();
    void statisticsTextChanged();
    void statusChanged();
    void modelRoiChanged();
    void modelRoiVisibleChanged();
    void modelRoiEditableChanged();
    void searchRoiChanged();
    void searchRoiEnabledChanged();
    void coarseMatchingRunningChanged();
    void coarseMatchFinishedChanged();
    void groundTruthChanged();

private:
    struct CoarseMatchUiResult
    {
        bool ok = false;
        QString message;
        ShapeMatch::CoarseMatchReport report;
        ShapeMatch::ShapeMatchOverlayData overlay;
    };

    void setStatus(const QString& status);
    void updateStatistics();
    void updateDisplayedLevelImage();
    void setCoarseMatchingRunning(bool running);
    void setCoarseMatchFinished(bool finished);
    bool setOption(bool* target, bool value);
    QRectF defaultSearchRoi() const;
    QRectF boundedImageRect(const QRectF& rect, double minSize) const;
    void createOrUpdateSearchRoiOnDisplay();
    void syncSearchRoiFromDisplay();

    QPointer<VisionDisplay::VisionDisplayItem> m_display;
    QImage m_templateImage;
    VisionTools::Matching::ShapeTemplateModel m_model;
    ShapeModelOverlayOptions m_options;
    int m_currentLevel = 0;
    QRectF m_modelRoi;
    QRectF m_searchRoi;
    bool m_modelRoiVisible = true;
    bool m_modelRoiEditable = true;
    bool m_searchRoiEnabled = false;
    bool m_coarseMatchingRunning = false;
    bool m_coarseMatchFinished = false;
    QString m_statisticsText;
    QString m_status = QStringLiteral("Load a template image.");
    QString m_lastCoarseSummary;
    QString m_groundTruthPath;
    QString m_groundTruthSummary;
    QPointF m_visionProTrainingOrigin;
    bool m_hasVisionProTrainingOrigin = false;

    QFutureWatcher<CoarseMatchUiResult> m_coarseWatcher;
};
