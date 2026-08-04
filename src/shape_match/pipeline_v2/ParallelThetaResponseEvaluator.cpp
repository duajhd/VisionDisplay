#include "shape_match/pipeline_v2/ParallelThetaResponseEvaluator.h"

#include "shape_match/coarse/ResponseMapEvaluator.h"
#include "shape_match/coarse/ResponsePeakExtractor.h"

#include <algorithm>
#include <future>
#include <thread>

namespace ShapeMatch {

namespace {

double thetaForBin(int bin, const OrientationResponseConfig& config)
{
    const int count = std::max(1, config.thetaBinCount);
    return degToRad(config.thetaMinDeg) +
           (static_cast<double>(bin) + 0.5) * degToRad(config.thetaMaxDeg - config.thetaMinDeg) /
               static_cast<double>(count);
}

int resolveThreadCount(const PipelineV2ProductionConfig& config, int workCount)
{
    if (!config.enableParallelThetaEvaluation || workCount <= 1) {
        return 1;
    }
    int threads = config.numWorkerThreads;
    if (threads <= 0) {
        threads = static_cast<int>(std::thread::hardware_concurrency());
    }
    return std::max(1, std::min(threads, workCount));
}

} // namespace

ParallelThetaResponseEvaluator::ParallelThetaResponseEvaluator(const PipelineV2ProductionConfig& productionConfig)
    : m_config(productionConfig)
{
}

std::vector<ResponsePeak> ParallelThetaResponseEvaluator::evaluateAllTheta(const OrientationResponseMap& responseMap,
                                                                          const ResponseTemplate& responseTemplate,
                                                                          const OrientationResponseConfig& responseConfig,
                                                                          ResponseDebugReport* debugReport,
                                                                          RuntimeBufferPool*)
                                                                          const
{
    const int thetaCount = std::max(1, responseConfig.thetaBinCount);
    const int threads = resolveThreadCount(m_config, thetaCount);
    ResponsePeakExtractor extractor;

    auto runRange = [&](int begin, int end) {
        ResponseMapEvaluator evaluator;
        std::vector<ResponsePeak> local;
        for (int thetaBin = begin; thetaBin < end; ++thetaBin) {
            const double thetaRad = thetaForBin(thetaBin, responseConfig);
            cv::Mat responseXY = evaluator.evaluateTheta(responseMap, responseTemplate, thetaBin, thetaRad, responseConfig);
            std::vector<ResponsePeak> peaks = extractor.extractPeaks(responseXY, thetaBin, thetaRad, responseConfig);
            local.insert(local.end(), peaks.begin(), peaks.end());
        }
        return local;
    };

    std::vector<ResponsePeak> allPeaks;
    if (threads == 1) {
        allPeaks = runRange(0, thetaCount);
    } else {
        std::vector<std::future<std::vector<ResponsePeak>>> futures;
        const int chunk = (thetaCount + threads - 1) / threads;
        for (int t = 0; t < threads; ++t) {
            const int begin = t * chunk;
            const int end = std::min(thetaCount, begin + chunk);
            if (begin < end) {
                futures.push_back(std::async(std::launch::async, runRange, begin, end));
            }
        }
        for (auto& f : futures) {
            std::vector<ResponsePeak> local = f.get();
            allPeaks.insert(allPeaks.end(), local.begin(), local.end());
        }
    }

    if (debugReport != nullptr) {
        debugReport->totalPeaksBeforeNms = static_cast<int>(allPeaks.size());
    }
    std::vector<ResponsePeak> finalPeaks = extractor.mergeAndNms(allPeaks, responseConfig);
    if (debugReport != nullptr) {
        debugReport->totalPeaksAfterNms = static_cast<int>(finalPeaks.size());
        debugReport->topPeaks = finalPeaks;
    }
    return finalPeaks;
}

} // namespace ShapeMatch
