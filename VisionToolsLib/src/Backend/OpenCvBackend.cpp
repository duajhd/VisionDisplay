#include "VisionTools/Backend/OpenCvBackend.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>

namespace VisionTools::Backend {

QString OpenCvBackend::openCvBuildInfo()
{
    const QString buildInfo = QString::fromStdString(cv::getBuildInformation());
    const int firstLineEnd = buildInfo.indexOf(QLatin1Char('\n'));
    const QString firstLine = firstLineEnd >= 0 ? buildInfo.left(firstLineEnd).trimmed() : buildInfo.trimmed();
    return QStringLiteral("OpenCV version=%1 build=%2").arg(QString::fromLatin1(CV_VERSION), firstLine);
}

bool OpenCvBackend::runOpenCvSmokeTest(QString& message)
{
    try {
        cv::Mat image(32, 32, CV_8UC1, cv::Scalar(0));
        cv::rectangle(image, cv::Rect(8, 8, 16, 16), cv::Scalar(220), cv::FILLED);

        cv::Mat blurred;
        cv::GaussianBlur(image, blurred, cv::Size(3, 3), 0.8);

        cv::Mat binary;
        cv::threshold(blurred, binary, 80.0, 255.0, cv::THRESH_BINARY);

        cv::Mat labels;
        cv::Mat stats;
        cv::Mat centroids;
        const int componentCount = cv::connectedComponentsWithStats(binary, labels, stats, centroids, 8, CV_32S);

        cv::Mat distance;
        cv::distanceTransform(binary, distance, cv::DIST_L2, 3);

        double minValue = 0.0;
        double maxValue = 0.0;
        cv::minMaxLoc(distance, &minValue, &maxValue);
        const bool ok = componentCount >= 2 && maxValue > 0.0 && labels.type() == CV_32S;
        message = QStringLiteral("OpenCV smoke %1: components=%2 distanceMax=%3")
                      .arg(ok ? QStringLiteral("OK") : QStringLiteral("NG"))
                      .arg(componentCount)
                      .arg(maxValue, 0, 'f', 3);
        return ok;
    } catch (const cv::Exception& ex) {
        message = QStringLiteral("OpenCV exception: %1").arg(QString::fromLocal8Bit(ex.what()));
        return false;
    } catch (const std::exception& ex) {
        message = QStringLiteral("OpenCV smoke exception: %1").arg(QString::fromLocal8Bit(ex.what()));
        return false;
    }
}

} // namespace VisionTools::Backend
