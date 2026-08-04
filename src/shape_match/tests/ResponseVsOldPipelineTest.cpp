#include "shape_match/coarse/OrientationVotingCandidateGenerator.h"
#include "shape_match/coarse/ResponseCandidateGenerator.h"
#include "shape_match/core/EdgeImageData.h"
#include "shape_match/core/ShapeTemplateModel.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

namespace ShapeMatch {

namespace {

struct MethodResult
{
    std::string caseName;
    std::string method;
    int level = 0;
    double timeMs = 0.0;
    int candidateCount = 0;
    int peakCount = 0;
    bool top1Hit = false;
    bool top3Hit = false;
    bool top5Hit = false;
    bool top10Hit = false;
    double recall = 0.0;
    int tp = 0;
    int fp = 0;
    int fn = 0;
    double avgDxy = 0.0;
    double avgDtheta = 0.0;
};

double elapsedMs(std::chrono::steady_clock::time_point start)
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

EdgeImageData createEmptyEdgeData(cv::Size size)
{
    EdgeImageData data;
    data.imageSize = size;
    data.edgeMap = cv::Mat::zeros(size, CV_8U);
    data.gradX = cv::Mat::zeros(size, CV_32F);
    data.gradY = cv::Mat::zeros(size, CV_32F);
    data.gradMag = cv::Mat::zeros(size, CV_32F);
    return data;
}

void addInstance(EdgeImageData& dst, const ShapeTemplateModel& model, const MatchPose& pose)
{
    EdgeImageData instance = EdgeImageData::createFromTemplateAndPose(model, pose, dst.imageSize);
    for (size_t i = 0; i < instance.edgePoints.size(); ++i) {
        const cv::Point2d p = instance.edgePoints[i];
        const cv::Point2d n = i < instance.edgeNormals.size() ? instance.edgeNormals[i] : cv::Point2d(1.0, 0.0);
        const int x = std::clamp(static_cast<int>(std::round(p.x)), 0, dst.imageSize.width - 1);
        const int y = std::clamp(static_cast<int>(std::round(p.y)), 0, dst.imageSize.height - 1);
        dst.edgePoints.emplace_back(x, y);
        dst.edgeNormals.push_back(n);
        dst.edgeMap.at<uchar>(y, x) = 255;
        dst.gradX.at<float>(y, x) = static_cast<float>(n.x);
        dst.gradY.at<float>(y, x) = static_cast<float>(n.y);
        dst.gradMag.at<float>(y, x) = 255.0f;
    }
}

double poseDxy(const MatchPose& a, const MatchPose& b)
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

double poseDtheta(const MatchPose& a, const MatchPose& b)
{
    return std::abs(radToDeg(wrapToPi(a.theta - b.theta)));
}

MethodResult evaluate(const std::string& caseName,
                      const std::string& method,
                      const std::vector<MatchPose>& candidates,
                      const std::vector<MatchPose>& gt,
                      double timeMs,
                      int peakCount)
{
    MethodResult r;
    r.caseName = caseName;
    r.method = method;
    r.timeMs = timeMs;
    r.candidateCount = static_cast<int>(candidates.size());
    r.peakCount = peakCount;
    auto topKHit = [&](int k) {
        const int count = std::min<int>(k, static_cast<int>(candidates.size()));
        for (int i = 0; i < count; ++i) {
            for (const MatchPose& g : gt) {
                if (poseDxy(candidates[i], g) <= 10.0 && poseDtheta(candidates[i], g) <= 12.0) {
                    return true;
                }
            }
        }
        return false;
    };
    r.top1Hit = topKHit(1);
    r.top3Hit = topKHit(3);
    r.top5Hit = topKHit(5);
    r.top10Hit = topKHit(10);

    double sumDxy = 0.0;
    double sumDtheta = 0.0;
    for (const MatchPose& g : gt) {
        double bestDxy = 1e9;
        double bestDtheta = 1e9;
        for (const MatchPose& c : candidates) {
            const double xy = poseDxy(c, g);
            if (xy < bestDxy) {
                bestDxy = xy;
                bestDtheta = poseDtheta(c, g);
            }
        }
        if (bestDxy <= 10.0) {
            ++r.tp;
            sumDxy += bestDxy;
            sumDtheta += bestDtheta;
        }
    }
    r.fn = static_cast<int>(gt.size()) - r.tp;
    r.fp = std::max(0, static_cast<int>(candidates.size()) - r.tp);
    r.recall = gt.empty() ? 0.0 : static_cast<double>(r.tp) / static_cast<double>(gt.size());
    r.avgDxy = r.tp > 0 ? sumDxy / r.tp : 0.0;
    r.avgDtheta = r.tp > 0 ? sumDtheta / r.tp : 0.0;
    return r;
}

void writeReports(const std::vector<MethodResult>& rows)
{
    const std::filesystem::path dir = std::filesystem::path("data") / "shape_match" / "reports";
    std::filesystem::create_directories(dir);
    {
        std::ofstream out(dir / "latest_response_vs_old_pipeline.csv");
        out << "case_name,method,level,time_ms,candidate_count,peak_count,top1_hit,top3_hit,top5_hit,top10_hit,recall,tp,fp,fn,avg_dxy,avg_dtheta\n";
        for (const MethodResult& r : rows) {
            out << r.caseName << ',' << r.method << ',' << r.level << ',' << r.timeMs << ','
                << r.candidateCount << ',' << r.peakCount << ',' << r.top1Hit << ','
                << r.top3Hit << ',' << r.top5Hit << ',' << r.top10Hit << ','
                << r.recall << ',' << r.tp << ',' << r.fp << ',' << r.fn << ','
                << r.avgDxy << ',' << r.avgDtheta << '\n';
        }
    }
    {
        std::ofstream out(dir / "latest_response_vs_old_pipeline.txt");
        out << "Response vs Old Pipeline\n";
        for (const MethodResult& r : rows) {
            out << r.caseName << " / " << r.method
                << ": timeMs=" << r.timeMs
                << " candidates=" << r.candidateCount
                << " recall=" << r.recall
                << " top10Hit=" << (r.top10Hit ? "true" : "false") << '\n';
        }
    }
}

} // namespace

int runResponseVsOldPipelineTest()
{
    const ShapeTemplateModel model = ShapeTemplateModel::createSyntheticRectangle("response_vs_old_rect", 48.0, 32.0, 18);
    EdgeImageData edgeData = createEmptyEdgeData(cv::Size(360, 260));
    const std::vector<MatchPose> gt{
        MatchPose::fromDeg(85.0, 70.0, -10.0),
        MatchPose::fromDeg(275.0, 70.0, 28.0),
        MatchPose::fromDeg(85.0, 195.0, 62.0),
        MatchPose::fromDeg(275.0, 195.0, -38.0)
    };
    for (const MatchPose& pose : gt) {
        addInstance(edgeData, model, pose);
    }

    VotingConfig votingConfig;
    votingConfig.topVotePeaks = 300;
    votingConfig.maxTemplateVotePoints = 120;
    votingConfig.maxImageVotePoints = 6000;
    votingConfig.enableVotingDebugReport = false;
    const auto oldStart = std::chrono::steady_clock::now();
    OrientationVotingCandidateGenerator oldGenerator(votingConfig);
    const std::vector<MatchPose> oldCandidates = oldGenerator.generate(model, edgeData, nullptr);
    const double oldMs = elapsedMs(oldStart);

    OrientationResponseConfig responseConfig;
    responseConfig.thetaBinCount = 36;
    responseConfig.maxTemplateResponsePoints = 80;
    responseConfig.maxTotalPeaks = 300;
    responseConfig.maxPeaksPerTheta = 80;
    responseConfig.minResponseScore = 0.10;
    const auto responseStart = std::chrono::steady_clock::now();
    ResponseDebugReport responseReport;
    ResponseCandidateGenerator responseGenerator(responseConfig);
    const std::vector<MatchPose> responseCandidates = responseGenerator.generate(model, edgeData, 0, &responseReport);
    const double responseMs = elapsedMs(responseStart);

    std::vector<MethodResult> rows;
    rows.push_back(evaluate("synthetic_four_targets", "old_orientation_voting", oldCandidates, gt, oldMs, static_cast<int>(oldCandidates.size())));
    rows.push_back(evaluate("synthetic_four_targets", "orientation_response", responseCandidates, gt, responseMs, responseReport.totalPeaksAfterNms));
    writeReports(rows);

    bool ok = true;
    ok &= rows.back().recall >= 0.75;
    ok &= responseReport.totalPeaksAfterNms <= responseConfig.maxTotalPeaks;
    ok &= std::filesystem::exists(std::filesystem::path("data") / "shape_match" / "reports" / "latest_response_vs_old_pipeline.csv");

    for (const MethodResult& r : rows) {
        std::cout << "[ResponseVsOld] method=" << r.method
                  << " timeMs=" << r.timeMs
                  << " candidates=" << r.candidateCount
                  << " recall=" << r.recall << '\n';
    }
    if (ok) {
        std::cout << "[ResponseVsOld] self-test PASSED\n";
    }
    return ok ? 0 : 1;
}

} // namespace ShapeMatch

int main()
{
    return ShapeMatch::runResponseVsOldPipelineTest();
}
