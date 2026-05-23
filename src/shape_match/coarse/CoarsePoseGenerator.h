#pragma once

#include "shape_match/coarse/CoarseMatchConfig.h"
#include "shape_match/coarse/FastPoseScorer.h"
#include "shape_match/coarse/TopKCandidateBuffer.h"
#include "shape_match/coarse/VotingDebugReport.h"

namespace ShapeMatch {

class CoarsePoseGenerator
{
public:
    explicit CoarsePoseGenerator(CoarseMatchConfig config = {});

    CoarseMatchLevelResult searchGlobal(const ShapeTemplateModel& templateLevel,
                                        const EdgeImageData& imageLevel,
                                        int level) const;

    CoarseMatchLevelResult searchVotingInitial(const ShapeTemplateModel& templateLevel,
                                               const EdgeImageData& imageLevel,
                                               int level,
                                               const VotingDebugReport& votingReport,
                                               const std::vector<MatchPose>& votingPoses) const;

    CoarseMatchLevelResult searchLocal(const ShapeTemplateModel& templateLevel,
                                       const EdgeImageData& imageLevel,
                                       int level,
                                       double currentLevelScale,
                                       double previousLevelScale,
                                       const std::vector<CoarseCandidate>& previousCandidates,
                                       const EdgeQueryContext* queryContext = nullptr,
                                       const std::vector<cv::Rect>* activeRegions = nullptr) const;

    static MatchPose convertPoseBetweenLevels(const MatchPose& pose,
                                              double fromLevelScale,
                                              double toLevelScale);

private:
    std::vector<double> scaleValues() const;
    double angleStepForLevel(int level) const;
    int translationStepForLevel(int level) const;

    CoarseMatchConfig m_config;
};

} // namespace ShapeMatch
