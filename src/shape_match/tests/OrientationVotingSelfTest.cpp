#include "shape_match/tests/OrientationVotingSelfTest.h"

#include "shape_match/coarse/ImageEdgeSampler.h"
#include "shape_match/coarse/OrientationVotingCandidateGenerator.h"
#include "shape_match/core/ShapeTemplateModel.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <set>

namespace ShapeMatch {

namespace {

bool expect(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "[OrientationVoting][FAIL] " << message << '\n';
    }
    return condition;
}

VotingConfig testConfig()
{
    VotingConfig config;
    config.orientationBinCount = 72;
    config.angleToleranceDeg = 7.5f;
    config.accumulatorCellSizePx = 4.0f;
    config.accumulatorNmsRadiusXY = 2;
    config.accumulatorNmsRadiusTheta = 1;
    config.maxTemplateVotePoints = 160;
    config.maxImageVotePoints = 5000;
    config.topVotePeaks = 160;
    config.minVoteCount = 3;
    config.imageTileRows = 12;
    config.imageTileCols = 12;
    config.maxImagePointsPerTile = 50;
    config.enableVotingDebugReport = true;
    return config;
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
        dst.edgePoints.push_back(p);
        dst.edgeNormals.push_back(n);
        const int x = std::clamp(static_cast<int>(std::round(p.x)), 0, dst.imageSize.width - 1);
        const int y = std::clamp(static_cast<int>(std::round(p.y)), 0, dst.imageSize.height - 1);
        dst.edgeMap.at<uchar>(y, x) = 255;
        dst.gradX.at<float>(y, x) = static_cast<float>(n.x);
        dst.gradY.at<float>(y, x) = static_cast<float>(n.y);
        dst.gradMag.at<float>(y, x) = 255.0f;
    }
}

void addLineInterference(EdgeImageData& data, const cv::Point2d& a, const cv::Point2d& b, int samples)
{
    const cv::Point2d dir = b - a;
    const cv::Point2d normal = normalized(cv::Point2d(-dir.y, dir.x));
    for (int i = 0; i < samples; ++i) {
        const double t = samples <= 1 ? 0.0 : static_cast<double>(i) / static_cast<double>(samples - 1);
        const cv::Point2d p = a + dir * t;
        if (!data.isInside(p)) {
            continue;
        }
        const int x = std::clamp(static_cast<int>(std::round(p.x)), 0, data.imageSize.width - 1);
        const int y = std::clamp(static_cast<int>(std::round(p.y)), 0, data.imageSize.height - 1);
        data.edgePoints.emplace_back(static_cast<double>(x), static_cast<double>(y));
        data.edgeNormals.push_back(normal);
        data.edgeMap.at<uchar>(y, x) = 255;
        data.gradX.at<float>(y, x) = static_cast<float>(normal.x);
        data.gradY.at<float>(y, x) = static_cast<float>(normal.y);
        data.gradMag.at<float>(y, x) = 255.0f;
    }
}

double angleDiffDeg(double a, double b)
{
    return std::abs(radToDeg(wrapToPi(a - b)));
}

bool hasPeakNear(const VotingDebugReport& report, const MatchPose& gtPose, double maxDxy, double maxDthetaDeg)
{
    for (const VotePeak& peak : report.topPeaks) {
        const double dx = peak.pose.x - gtPose.x;
        const double dy = peak.pose.y - gtPose.y;
        const double dxy = std::sqrt(dx * dx + dy * dy);
        if (dxy <= maxDxy && angleDiffDeg(peak.pose.theta, gtPose.theta) <= maxDthetaDeg) {
            return true;
        }
    }
    return false;
}

bool hasSpatialPeakNear(const VotingDebugReport& report, const MatchPose& gtPose, double maxDxy)
{
    for (const VotePeak& peak : report.topPeaks) {
        const double dx = peak.pose.x - gtPose.x;
        const double dy = peak.pose.y - gtPose.y;
        if (std::sqrt(dx * dx + dy * dy) <= maxDxy) {
            return true;
        }
    }
    return false;
}

int occupiedQuadrants(const VotingDebugReport& report, cv::Size imageSize)
{
    std::set<std::pair<int, int>> cells;
    for (const VotePeak& peak : report.topPeaks) {
        const int col = peak.pose.x < imageSize.width * 0.5 ? 0 : 1;
        const int row = peak.pose.y < imageSize.height * 0.5 ? 0 : 1;
        cells.insert({row, col});
    }
    return static_cast<int>(cells.size());
}

void printSummary(const char* name, const VotingDebugReport& report)
{
    std::cout << "[OrientationVoting] case=" << name
              << " sampled=" << report.sampledImageVotePoints
              << " votes=" << report.totalVotes
              << " bins=" << report.accumulatorNonZeroBins
              << " peaks=" << report.peakCount
              << " totalMs=" << report.totalMs << '\n';
    for (size_t i = 0; i < report.topPeaks.size() && i < 5; ++i) {
        const VotePeak& p = report.topPeaks[i];
        std::cout << "  peak " << (i + 1)
                  << " pose=(" << p.pose.x << ',' << p.pose.y << ',' << p.pose.thetaDeg() << ")"
                  << " votes=" << p.voteCount
                  << " score=" << p.voteScore << '\n';
    }
}

} // namespace

bool runOrientationVotingSelfTest()
{
    bool ok = true;
    const ShapeTemplateModel model = ShapeTemplateModel::createSyntheticRectangle("voting_rect", 100.0, 70.0, 28);
    const VotingConfig config = testConfig();
    const double maxDxy = static_cast<double>(config.accumulatorCellSizePx) * 2.0;
    const double maxDtheta = 360.0 / static_cast<double>(config.orientationBinCount) + 0.25;

    {
        const MatchPose gt = MatchPose::fromDeg(320.0, 240.0, 0.0);
        const EdgeImageData edgeData = EdgeImageData::createFromTemplateAndPose(model, gt, cv::Size(640, 480));
        VotingDebugReport report;
        OrientationVotingCandidateGenerator generator(config);
        const std::vector<MatchPose> candidates = generator.generate(model, edgeData, &report);
        printSummary("single_no_rotation", report);
        ok &= expect(!candidates.empty(), "single no-rotation should produce candidates");
        ok &= expect(hasPeakNear(report, gt, maxDxy, maxDtheta), "single no-rotation should have a GT-near voting peak");
    }

    {
        const MatchPose gt = MatchPose::fromDeg(330.0, 250.0, 37.0);
        const EdgeImageData edgeData = EdgeImageData::createFromTemplateAndPose(model, gt, cv::Size(700, 520));
        VotingDebugReport report;
        OrientationVotingCandidateGenerator generator(config);
        generator.generate(model, edgeData, &report);
        printSummary("single_rotation", report);
        ok &= expect(hasPeakNear(report, gt, maxDxy, maxDtheta), "single rotation should have a GT-near voting peak");
    }

    {
        const cv::Size imageSize(860, 640);
        EdgeImageData edgeData = createEmptyEdgeData(imageSize);
        const std::vector<MatchPose> gt{
            MatchPose::fromDeg(185.0, 150.0, -12.0),
            MatchPose::fromDeg(675.0, 150.0, 26.0),
            MatchPose::fromDeg(185.0, 490.0, 64.0),
            MatchPose::fromDeg(675.0, 490.0, -38.0)
        };
        for (const MatchPose& pose : gt) {
            addInstance(edgeData, model, pose);
        }
        VotingDebugReport report;
        OrientationVotingCandidateGenerator generator(config);
        generator.generate(model, edgeData, &report);
        printSummary("multi_target", report);
        int hitCount = 0;
        for (const MatchPose& pose : gt) {
            if (hasSpatialPeakNear(report, pose, maxDxy)) {
                ++hitCount;
            }
        }
        ok &= expect(hitCount >= 3, "multi-target voting should find peaks near at least three GT objects");
        ok &= expect(occupiedQuadrants(report, imageSize) >= 3, "multi-target peaks should cover multiple spatial regions");
    }

    {
        const MatchPose gt = MatchPose::fromDeg(360.0, 275.0, -22.0);
        EdgeImageData edgeData = EdgeImageData::createFromTemplateAndPose(model, gt, cv::Size(760, 560));
        addLineInterference(edgeData, cv::Point2d(40.0, 500.0), cv::Point2d(720.0, 70.0), 1000);
        VotingDebugReport report;
        OrientationVotingCandidateGenerator generator(config);
        generator.generate(model, edgeData, &report);
        printSummary("strong_line_interference", report);
        ok &= expect(hasSpatialPeakNear(report, gt, maxDxy), "strong line interference should not remove the true spatial peak");
    }

    {
        EdgeImageData edgeData = createEmptyEdgeData(cv::Size(800, 500));
        addLineInterference(edgeData, cv::Point2d(30.0, 20.0), cv::Point2d(30.0, 480.0), 1600);
        const MatchPose weakTarget = MatchPose::fromDeg(650.0, 250.0, 15.0);
        EdgeImageData target = EdgeImageData::createFromTemplateAndPose(model, weakTarget, edgeData.imageSize);
        for (size_t i = 0; i < target.edgePoints.size(); ++i) {
            const cv::Point2d p = target.edgePoints[i];
            const cv::Point2d n = i < target.edgeNormals.size() ? target.edgeNormals[i] : cv::Point2d(1.0, 0.0);
            edgeData.edgePoints.push_back(p);
            edgeData.edgeNormals.push_back(n);
            const int x = std::clamp(static_cast<int>(std::round(p.x)), 0, edgeData.imageSize.width - 1);
            const int y = std::clamp(static_cast<int>(std::round(p.y)), 0, edgeData.imageSize.height - 1);
            edgeData.edgeMap.at<uchar>(y, x) = 255;
            edgeData.gradX.at<float>(y, x) = static_cast<float>(n.x);
            edgeData.gradY.at<float>(y, x) = static_cast<float>(n.y);
            edgeData.gradMag.at<float>(y, x) = 60.0f;
        }
        ImageEdgeSamplingStats stats;
        ImageEdgeSampler sampler;
        const std::vector<ImageVotePoint> sampled = sampler.sample(edgeData, config, &stats);
        bool hasWeakSidePoint = false;
        for (const ImageVotePoint& p : sampled) {
            if (p.x > 560.0f && p.y > 170.0f && p.y < 330.0f) {
                hasWeakSidePoint = true;
                break;
            }
        }
        std::cout << "[OrientationVoting] case=tile_sampling raw=" << stats.rawEdgeCount
                  << " sampled=" << stats.sampledCount
                  << " tileMin=" << stats.minTileCount
                  << " tileMax=" << stats.maxTileCount << '\n';
        ok &= expect(hasWeakSidePoint, "tile sampling should preserve points from the weak target region");
    }

    std::cout << "[OrientationVoting] reports_dir="
              << (std::filesystem::path("data") / "shape_match" / "reports").string() << '\n';
    if (ok) {
        std::cout << "[OrientationVoting] self-test PASSED\n";
    }
    return ok;
}

} // namespace ShapeMatch
