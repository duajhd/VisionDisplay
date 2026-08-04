#include "shape_match/coarse/ResponseCandidateGenerator.h"

#include "shape_match/coarse/OrientationResponseMapBuilder.h"
#include "shape_match/coarse/ResponseMapEvaluator.h"
#include "shape_match/coarse/ResponsePeakExtractor.h"
#include "shape_match/coarse/ResponseTemplate.h"
#include "shape_match/io/ResponseDebugReportWriter.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace ShapeMatch {

namespace {

double elapsedMs(std::chrono::steady_clock::time_point start)
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

double thetaForBin(int bin, const OrientationResponseConfig& config)
{
    const int count = std::max(1, config.thetaBinCount);
    const double minRad = degToRad(config.thetaMinDeg);
    const double maxRad = degToRad(config.thetaMaxDeg);
    return minRad + (static_cast<double>(bin) + 0.5) * (maxRad - minRad) / static_cast<double>(count);
}

void updateMaxDebugMaps(const cv::Mat& responseXY, int thetaBin, cv::Mat& maxResponse, cv::Mat& bestTheta)
{
    if (responseXY.empty()) {
        return;
    }
    if (maxResponse.empty()) {
        maxResponse = responseXY.clone();
        bestTheta = cv::Mat(responseXY.size(), CV_32F, cv::Scalar(static_cast<float>(thetaBin)));
        return;
    }
    for (int y = 0; y < responseXY.rows; ++y) {
        const float* src = responseXY.ptr<float>(y);
        float* maxRow = maxResponse.ptr<float>(y);
        float* thetaRow = bestTheta.ptr<float>(y);
        for (int x = 0; x < responseXY.cols; ++x) {
            if (src[x] > maxRow[x]) {
                maxRow[x] = src[x];
                thetaRow[x] = static_cast<float>(thetaBin);
            }
        }
    }
}

} // namespace

ResponseCandidateGenerator::ResponseCandidateGenerator(OrientationResponseConfig config)
    : m_config(std::move(config))
{
}

std::vector<MatchPose> ResponseCandidateGenerator::generate(const ShapeTemplateModel& templateModel,
                                                            const EdgeImageData& edgeData,
                                                            int pyramidLevel,
                                                            ResponseDebugReport* debugReport) const
{
    const std::vector<ResponsePeak> peaks = generatePeaks(templateModel, edgeData, pyramidLevel, debugReport);
    std::vector<MatchPose> poses;
    poses.reserve(peaks.size());
    for (const ResponsePeak& peak : peaks) {
        poses.push_back(peak.pose);
    }
    return poses;
}

std::vector<ResponsePeak> ResponseCandidateGenerator::generatePeaks(const ShapeTemplateModel& templateModel,
                                                                    const EdgeImageData& edgeData,
                                                                    int pyramidLevel,
                                                                    ResponseDebugReport* debugReport) const
{
    const auto totalStart = std::chrono::steady_clock::now();
    ResponseDebugReport localReport;
    localReport.level = pyramidLevel;
    localReport.imageWidth = edgeData.imageSize.width;
    localReport.imageHeight = edgeData.imageSize.height;
    localReport.orientationBinCount = m_config.orientationBinCount;
    localReport.thetaBinCount = m_config.thetaBinCount;
    localReport.edgeOverlayBase = edgeData.edgeMap;

    OrientationResponseMapBuilder mapBuilder;
    const OrientationResponseMap responseMap = mapBuilder.build(edgeData, pyramidLevel, m_config);
    localReport.buildResponseMapMs = responseMap.buildTimeMs;
    localReport.rawEdgeCount = responseMap.rawEdgeCount;
    localReport.usedEdgeCount = responseMap.usedEdgeCount;
    if (responseMap.empty()) {
        localReport.totalMs = elapsedMs(totalStart);
        if (debugReport != nullptr) {
            *debugReport = localReport;
        }
        ResponseDebugReportWriter().write(localReport, m_config);
        return {};
    }

    const auto templateStart = std::chrono::steady_clock::now();
    ResponseTemplateBuilder templateBuilder;
    const ResponseTemplate responseTemplate = templateBuilder.build(templateModel, pyramidLevel, m_config);
    localReport.buildTemplateMs = elapsedMs(templateStart);
    localReport.responseTemplatePointCount = responseTemplate.size();
    if (responseTemplate.empty()) {
        localReport.totalMs = elapsedMs(totalStart);
        if (debugReport != nullptr) {
            *debugReport = localReport;
        }
        ResponseDebugReportWriter().write(localReport, m_config);
        return {};
    }

    ResponseMapEvaluator evaluator;
    ResponsePeakExtractor extractor;
    std::vector<ResponsePeak> allPeaks;
    allPeaks.reserve(static_cast<size_t>(std::min(m_config.maxTotalPeaks * 2, m_config.maxPeaksPerTheta * m_config.thetaBinCount)));

    for (int thetaBin = 0; thetaBin < std::max(1, m_config.thetaBinCount); ++thetaBin) {
        const double thetaRad = thetaForBin(thetaBin, m_config);
        const auto evalStart = std::chrono::steady_clock::now();
        cv::Mat responseXY = evaluator.evaluateTheta(responseMap, responseTemplate, thetaBin, thetaRad, m_config);
        localReport.evaluateResponseMs += elapsedMs(evalStart);
        updateMaxDebugMaps(responseXY, thetaBin, localReport.maxResponseXY, localReport.bestThetaXY);

        const auto peakStart = std::chrono::steady_clock::now();
        std::vector<ResponsePeak> peaks = extractor.extractPeaks(responseXY, thetaBin, thetaRad, m_config);
        localReport.peakExtractMs += elapsedMs(peakStart);
        localReport.totalPeaksBeforeNms += static_cast<int>(peaks.size());
        allPeaks.insert(allPeaks.end(), peaks.begin(), peaks.end());
    }

    const auto nmsStart = std::chrono::steady_clock::now();
    std::vector<ResponsePeak> finalPeaks = extractor.mergeAndNms(allPeaks, m_config);
    localReport.peakExtractMs += elapsedMs(nmsStart);
    localReport.totalPeaksAfterNms = static_cast<int>(finalPeaks.size());
    localReport.topPeaks = finalPeaks;
    localReport.totalMs = elapsedMs(totalStart);

    if (debugReport != nullptr) {
        *debugReport = localReport;
    }
    ResponseDebugReportWriter().write(localReport, m_config);
    return finalPeaks;
}

} // namespace ShapeMatch
