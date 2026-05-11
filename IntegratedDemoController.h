#pragma once

#include <QObject>
#include <QImage>
#include <QPointer>
#include <QString>
#include <QVariantMap>

namespace VisionDisplay {
class VisionDisplayItem;
}

class IntegratedDemoController : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString status READ status NOTIFY statusChanged)

public:
    explicit IntegratedDemoController(QObject* parent = nullptr);

    QString status() const;

    Q_INVOKABLE void bindDisplay(QObject* displayObject);
    Q_INVOKABLE void resetImage();
    Q_INVOKABLE void resetEllipseImage();
    Q_INVOKABLE void generateCircleTestImage();
    Q_INVOKABLE bool loadImage(const QString& pathOrUrl);
    Q_INVOKABLE void createFindCircleCalipers();
    Q_INVOKABLE void runCaliper();
    Q_INVOKABLE void runFindLine();
    Q_INVOKABLE void runFindCircle();
    Q_INVOKABLE void runFindCircleRegression();
    Q_INVOKABLE void runFindCircleRegressionRobustOff();
    Q_INVOKABLE void runFindCircleRegressionCurrentSettings();
    Q_INVOKABLE QVariantMap runSingleCaliperDebug(double centerX,
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
                                                  int fallbackEdgeSelection);
    Q_INVOKABLE void clearSingleCaliperDebugGraphics();
    Q_INVOKABLE QString lastSingleCaliperDiagnostics() const;
    Q_INVOKABLE void runFindEllipse();
    Q_INVOKABLE void runFindEllipseWithLoss(const QString& lossType);
    Q_INVOKABLE void clearCaliperGraphics();
    Q_INVOKABLE void clearFindLineGraphics();
    Q_INVOKABLE void clearFindCircleGraphics();
    Q_INVOKABLE void clearFindEllipseGraphics();
    Q_INVOKABLE void clearToolGraphics();

signals:
    void statusChanged();

private:
    enum class DemoImageKind
    {
        CircleSynthetic,
        EllipseSynthetic,
        External
    };

    QImage createSyntheticImage() const;
    QImage createCircleRegressionImage() const;
    QImage createEllipseImage() const;
    QString localFilePath(const QString& pathOrUrl) const;
    void createSyntheticCircleCalipers();
    void createRegressionCircleCalipers();
    void runFindCircleRegressionInternal(bool robustOff, const QString& label);
    void setStatus(const QString& status);

    QPointer<VisionDisplay::VisionDisplayItem> m_display;
    QImage m_image;
    DemoImageKind m_imageKind = DemoImageKind::CircleSynthetic;
    QString m_status;
    QString m_circleDiagnostics;
    QString m_singleCaliperDiagnostics;
};
