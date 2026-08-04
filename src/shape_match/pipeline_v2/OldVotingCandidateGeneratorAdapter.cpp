#include "shape_match/pipeline_v2/OldVotingCandidateGeneratorAdapter.h"

#include "shape_match/coarse/OrientationVotingCandidateGenerator.h"
#include "shape_match/coarse/VotingConfig.h"

#include <algorithm>
#include <chrono>

namespace ShapeMatch {

namespace {

const ShapeTemplateModel* selectTemplateModel(const ShapeMatchPipelineV2Context& context, int level)
{
    if (context.templatePyramid != nullptr) {
        if (level >= 0 && level < context.templatePyramid->levelCount()) {
            return &context.templatePyramid->level(level);
        }
        return nullptr;
    }
    return level == 0 ? context.templateModel : nullptr;
}

const EdgeImageData* selectEdgeData(const ShapeMatchPipelineV2Context& context, int level)
{
    if (context.imagePyramid != nullptr) {
        if (level >= 0 && level < context.imagePyramid->levelCount()) {
            return &context.imagePyramid->level(level);
        }
        return nullptr;
    }
    return level == 0 ? context.edgeData : nullptr;
}

double elapsedMs(std::chrono::steady_clock::time_point start)
{
    const auto end = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(end - start).count();
}

} // namespace

std::vector<MatchPose> OldVotingCandidateGeneratorAdapter::generate(const ShapeMatchPipelineV2Context& context,
                                                                    const ShapeMatchPipelineV2Config& config,
                                                                    ShapeMatchPipelineV2Result* result)
{
    const auto start = std::chrono::steady_clock::now();
    std::vector<MatchPose> candidates;

    const ShapeTemplateModel* model = selectTemplateModel(context, config.pyramidLevel);
    const EdgeImageData* edgeData = selectEdgeData(context, config.pyramidLevel);
    if (model == nullptr || edgeData == nullptr) {
        if (result != nullptr) {
            result->candidateGenerationMs = elapsedMs(start);
            result->failureReason = "Invalid pyramid level or missing pyramid data for OldVoting";
        }
        return candidates;
    }

    VotingConfig votingConfig;
    votingConfig.enableVotingDebugReport = config.enableDebugReport;
    votingConfig.topVotePeaks = std::max(1, config.maxCandidates);

    OrientationVotingCandidateGenerator generator(votingConfig);
    candidates = generator.generate(*model, *edgeData, nullptr);

    if (static_cast<int>(candidates.size()) > config.maxCandidates) {
        candidates.resize(std::max(0, config.maxCandidates));
    }

    if (result != nullptr) {
        result->candidateGenerationMs = elapsedMs(start);
    }
    return candidates;
}

const char* OldVotingCandidateGeneratorAdapter::name() const
{
    return "OldVotingCandidateGeneratorAdapter";
}

} // namespace ShapeMatch
