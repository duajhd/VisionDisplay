#include "shape_match/coarse/OrientationVotingCandidateGenerator.h"

#include "shape_match/coarse/ImageEdgeSampler.h"
#include "shape_match/coarse/RTTable.h"

#include <chrono>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>

namespace ShapeMatch {

namespace {

double elapsedMs(std::chrono::steady_clock::time_point start, std::chrono::steady_clock::time_point end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

float normalizedMagnitudeWeight(float mag)
{
    return std::clamp(mag / 255.0f, 0.05f, 1.5f);
}

} // namespace

OrientationVotingCandidateGenerator::OrientationVotingCandidateGenerator(VotingConfig config)
    : m_config(config)
{
}

std::vector<MatchPose> OrientationVotingCandidateGenerator::generate(const ShapeTemplateModel& model,
                                                                     const EdgeImageData& edgeData,
                                                                     VotingDebugReport* debugReport) const
{
    VotingDebugReport localReport;
    auto totalStart = std::chrono::steady_clock::now();
    localReport.templatePointCount = model.pointCount();

    auto t0 = std::chrono::steady_clock::now();
    RTTable rtTable;
    if (!rtTable.build(model, m_config)) {
        if (debugReport) {
            *debugReport = localReport;
        }
        return {};
    }
    auto t1 = std::chrono::steady_clock::now();
    localReport.buildRtTableMs = elapsedMs(t0, t1);
    localReport.selectedTemplateVotePoints = rtTable.selectedTemplatePointCount();
    localReport.rtEntryCount = rtTable.totalEntryCount();

    ImageEdgeSamplingStats samplingStats;
    ImageEdgeSampler sampler;
    t0 = std::chrono::steady_clock::now();
    const std::vector<ImageVotePoint> imagePoints = sampler.sample(edgeData, m_config, &samplingStats);
    t1 = std::chrono::steady_clock::now();
    localReport.sampleImageEdgesMs = elapsedMs(t0, t1);
    localReport.rawImageEdgeCount = samplingStats.rawEdgeCount;
    localReport.sampledImageVotePoints = samplingStats.sampledCount;
    localReport.minTileSampledCount = samplingStats.minTileCount;
    localReport.maxTileSampledCount = samplingStats.maxTileCount;
    localReport.avgTileSampledCount = samplingStats.avgTileCount;

    SparseAccumulator accumulator(m_config);
    t0 = std::chrono::steady_clock::now();
    for (const ImageVotePoint& point : imagePoints) {
        const std::vector<RTEntry>& entries = rtTable.entriesForImageGradientBin(point.gradBin);
        const float imageWeight = m_config.useGradientMagnitudeWeight ? normalizedMagnitudeWeight(point.gradMag) : 1.0f;
        for (const RTEntry& entry : entries) {
            const float centerX = point.x - entry.dx;
            const float centerY = point.y - entry.dy;
            accumulator.vote(centerX, centerY, entry.thetaBin, std::max(0.01f, entry.weight * imageWeight));
        }
    }
    t1 = std::chrono::steady_clock::now();
    localReport.votingMs = elapsedMs(t0, t1);
    localReport.accumulatorNonZeroBins = accumulator.nonZeroBinCount();
    localReport.totalVotes = accumulator.totalVoteCount();

    t0 = std::chrono::steady_clock::now();
    localReport.topPeaks = accumulator.findPeaks(m_config.topVotePeaks, edgeData.imageSize);
    t1 = std::chrono::steady_clock::now();
    localReport.peakFindMs = elapsedMs(t0, t1);
    localReport.peakCount = static_cast<int>(localReport.topPeaks.size());
    localReport.totalMs = elapsedMs(totalStart, std::chrono::steady_clock::now());

    std::vector<MatchPose> poses;
    poses.reserve(localReport.topPeaks.size());
    for (const VotePeak& peak : localReport.topPeaks) {
        poses.push_back(peak.pose);
    }

    if (m_config.enableVotingDebugReport) {
        writeDebugReport(localReport);
    }
    if (debugReport) {
        *debugReport = localReport;
    }
    return poses;
}

void OrientationVotingCandidateGenerator::writeDebugReport(const VotingDebugReport& report) const
{
    try {
        const std::filesystem::path reportsDir = std::filesystem::path("data") / "shape_match" / "reports";
        std::filesystem::create_directories(reportsDir);

        nlohmann::json root;
        root["templatePointCount"] = report.templatePointCount;
        root["selectedTemplateVotePoints"] = report.selectedTemplateVotePoints;
        root["rtEntryCount"] = report.rtEntryCount;
        root["rawImageEdgeCount"] = report.rawImageEdgeCount;
        root["sampledImageVotePoints"] = report.sampledImageVotePoints;
        root["minTileSampledCount"] = report.minTileSampledCount;
        root["maxTileSampledCount"] = report.maxTileSampledCount;
        root["avgTileSampledCount"] = report.avgTileSampledCount;
        root["accumulatorNonZeroBins"] = report.accumulatorNonZeroBins;
        root["totalVotes"] = report.totalVotes;
        root["peakCount"] = report.peakCount;
        root["buildRtTableMs"] = report.buildRtTableMs;
        root["sampleImageEdgesMs"] = report.sampleImageEdgesMs;
        root["votingMs"] = report.votingMs;
        root["peakFindMs"] = report.peakFindMs;
        root["totalMs"] = report.totalMs;
        for (const VotePeak& peak : report.topPeaks) {
            root["topPeaks"].push_back({
                {"x", peak.pose.x},
                {"y", peak.pose.y},
                {"thetaDeg", peak.pose.thetaDeg()},
                {"voteScore", peak.voteScore},
                {"voteCount", peak.voteCount},
                {"xBin", peak.xBin},
                {"yBin", peak.yBin},
                {"thetaBin", peak.thetaBin}
            });
        }
        {
            std::ofstream file(reportsDir / "latest_voting_debug.json");
            file << std::setw(2) << root;
        }
        {
            std::ofstream file(reportsDir / "latest_voting_peaks.csv");
            file << "rank,x,y,theta_deg,vote_score,vote_count,x_bin,y_bin,theta_bin\n";
            int rank = 1;
            for (const VotePeak& peak : report.topPeaks) {
                file << rank++ << ','
                     << peak.pose.x << ',' << peak.pose.y << ',' << peak.pose.thetaDeg() << ','
                     << peak.voteScore << ',' << peak.voteCount << ','
                     << peak.xBin << ',' << peak.yBin << ',' << peak.thetaBin << '\n';
            }
        }
    } catch (...) {
    }
}

} // namespace ShapeMatch
