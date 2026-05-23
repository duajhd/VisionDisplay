#include "VisionTools/Backend/DependencySelfTest.h"

#include "VisionTools/Backend/CeresBackend.h"
#include "VisionTools/Backend/OpenCvBackend.h"

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <QStringList>

#include <cmath>

namespace VisionTools::Backend {

namespace {

bool runEigenSmokeTest(QString& message)
{
    Eigen::Matrix2d a;
    a << 3.0, 1.0,
         1.0, 2.0;
    Eigen::Vector2d b;
    b << 9.0, 8.0;
    const Eigen::Vector2d x = a.ldlt().solve(b);
    const double residual = (a * x - b).norm();
    const bool ok = std::isfinite(residual) && residual < 1.0e-10;
    message = QStringLiteral("Eigen smoke %1: version=%2.%3.%4 x=(%5,%6) residual=%7")
                  .arg(ok ? QStringLiteral("OK") : QStringLiteral("NG"))
                  .arg(EIGEN_WORLD_VERSION)
                  .arg(EIGEN_MAJOR_VERSION)
                  .arg(EIGEN_MINOR_VERSION)
                  .arg(x.x(), 0, 'f', 6)
                  .arg(x.y(), 0, 'f', 6)
                  .arg(residual, 0, 'e', 3);
    return ok;
}

bool runJsonSmokeTest(QString& message)
{
    nlohmann::json json;
    json["name"] = "VisionTools";
    json["ok"] = true;
    json["values"] = {1, 2, 3};
    const std::string dumped = json.dump();
    const nlohmann::json parsed = nlohmann::json::parse(dumped);
    const bool ok = parsed.value("ok", false)
        && parsed.value("name", std::string()) == "VisionTools"
        && parsed["values"].size() == 3;
    message = QStringLiteral("nlohmann_json smoke %1: version=%2.%3.%4 bytes=%5")
                  .arg(ok ? QStringLiteral("OK") : QStringLiteral("NG"))
                  .arg(NLOHMANN_JSON_VERSION_MAJOR)
                  .arg(NLOHMANN_JSON_VERSION_MINOR)
                  .arg(NLOHMANN_JSON_VERSION_PATCH)
                  .arg(static_cast<int>(dumped.size()));
    return ok;
}

bool runSpdlogSmokeTest(QString& message)
{
    try {
        spdlog::info("VisionTools dependency self test: spdlog OK");
        const bool ok = spdlog::default_logger_raw() != nullptr;
        message = QStringLiteral("spdlog smoke %1: version=%2.%3.%4")
                      .arg(ok ? QStringLiteral("OK") : QStringLiteral("NG"))
                      .arg(SPDLOG_VER_MAJOR)
                      .arg(SPDLOG_VER_MINOR)
                      .arg(SPDLOG_VER_PATCH);
        return ok;
    } catch (const std::exception& ex) {
        message = QStringLiteral("spdlog exception: %1").arg(QString::fromLocal8Bit(ex.what()));
        return false;
    }
}

} // namespace

QString DependencySelfTest::run()
{
    QStringList lines;
    int okCount = 0;
    int totalCount = 0;

    lines << QStringLiteral("Dependency Self Test");
    lines << OpenCvBackend::openCvBuildInfo();
    lines << CeresBackend::ceresBuildInfo();

    auto appendTest = [&](const QString& name, bool ok, const QString& message) {
        ++totalCount;
        if (ok) {
            ++okCount;
        }
        lines << QStringLiteral("%1: %2").arg(name, message);
    };

    QString message;
    appendTest(QStringLiteral("OpenCV"), OpenCvBackend::runOpenCvSmokeTest(message), message);
    appendTest(QStringLiteral("Ceres"), CeresBackend::runCeresSmokeTest(message), message);
    appendTest(QStringLiteral("Eigen"), runEigenSmokeTest(message), message);
    appendTest(QStringLiteral("nlohmann_json"), runJsonSmokeTest(message), message);
    appendTest(QStringLiteral("spdlog"), runSpdlogSmokeTest(message), message);

    lines << QStringLiteral("summary okCount=%1/%2").arg(okCount).arg(totalCount);
    return lines.join(QLatin1Char('\n'));
}

} // namespace VisionTools::Backend
