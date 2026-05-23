#include "VisionTools/Backend/CeresBackend.h"

#include <ceres/ceres.h>
#include <ceres/version.h>

#include <cmath>
#include <memory>

namespace VisionTools::Backend {

namespace {

struct ShiftCost
{
    template <typename T>
    bool operator()(const T* const x, T* residual) const
    {
        residual[0] = x[0] - T(10.0);
        return true;
    }
};

} // namespace

QString CeresBackend::ceresBuildInfo()
{
    return QStringLiteral("Ceres version=%1").arg(QString::fromLatin1(CERES_VERSION_STRING));
}

bool CeresBackend::runCeresSmokeTest(QString& message)
{
    double x = 0.0;
    ceres::Problem problem;
    auto* cost = new ceres::AutoDiffCostFunction<ShiftCost, 1, 1>(new ShiftCost);
    problem.AddResidualBlock(cost, nullptr, &x);

    ceres::Solver::Options options;
    options.max_num_iterations = 25;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    const bool ok = summary.IsSolutionUsable() && std::abs(x - 10.0) < 1.0e-6;
    message = QStringLiteral("Ceres smoke %1: x=%2 report=%3")
                  .arg(ok ? QStringLiteral("OK") : QStringLiteral("NG"))
                  .arg(x, 0, 'f', 9)
                  .arg(QString::fromStdString(summary.BriefReport()));
    return ok;
}

} // namespace VisionTools::Backend
