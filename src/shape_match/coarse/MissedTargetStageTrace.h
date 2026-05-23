#pragma once

#include "shape_match/core/ShapeMatchTypes.h"
#include "shape_match/overlay/ShapeMatchOverlayData.h"

#include <opencv2/core.hpp>

#include <filesystem>
#include <string>
#include <vector>

namespace ShapeMatch {

struct CoarseMatchReport;
struct CoarseMatchConfig;
class ImagePyramid;
class TemplatePyramid;

struct CandidateTraceInfo
{
    int candidateId = -1;
    int suppressorCandidateId = -1;
    std::string source;
    int level = -1;
    int rank = -1;
    MatchPose pose;
    double dxy = 0.0;
    double dthetaDeg = 0.0;
    double voteScore = 0.0;
    int voteCount = 0;
    double fastScore = 0.0;
    double finalScore = 0.0;
    double coverageApprox = 0.0;
    double orientationApprox = 0.0;
    double polarityApprox = 0.0;
    bool exists = false;
    bool kept = false;
    bool suppressedByNms = false;
    bool clippedByBudget = false;
    bool prunedByUpperBound = false;
    bool droppedByParentSelection = false;
    bool droppedByParallelMerge = false;
    std::string rejectReason;
    int gridCellRow = -1;
    int gridCellCol = -1;
};

struct StageTraceSnapshot
{
    std::string stage;
    int level = -1;
    std::vector<CandidateTraceInfo> candidates;
};

struct RoiVoteTraceStats
{
    int rawEdgeCountInRoi = 0;
    int sampledVotePointCountInRoi = 0;
    double votePointKeepRatioInRoi = 0.0;
    double roiVoteContribution = 0.0;
    double roiNearestVotingPeakDxy = 0.0;
    double roiNearestVotingPeakDtheta = 0.0;
    double avgGradMag = 0.0;
    double maxGradMag = 0.0;
    std::vector<int> gradientBinHistogram;
};

struct MissedTargetStageInfo
{
    std::string targetId;
    MatchPose expectedPose;
    cv::Rect2d debugRoi;
    bool hitFinalTopK = false;
    bool hitFinalRanked = false;
    std::string firstMissingStage;
    std::string likelyReason;
    std::vector<CandidateTraceInfo> nearestByStage;
    double oracleFastScoreLevel0 = 0.0;
    double oracleFastScoreTopLevel = 0.0;
    double oracleCoverageApproxLevel0 = 0.0;
    double oracleOrientationApproxLevel0 = 0.0;
    double oraclePolarityApproxLevel0 = 0.0;
    int oracleMatchedPointsLevel0 = 0;
    int oracleEvaluatedPointsLevel0 = 0;
    RoiVoteTraceStats roiVoteStats;
    int rawEdgeCountInRoi = 0;
    int sampledVotePointCountInRoi = 0;
    int votingPeakCountNearTarget = 0;
    int candidateCountNearTargetByLevel[8] = {};
    std::string diagnosis;
};

struct MissedTargetStageTraceReport
{
    std::string imageName;
    std::string templateName;
    std::vector<MissedTargetStageInfo> targets;
    double nearDxyThresholdPx = 10.0;
    double nearAngleThresholdDeg = 10.0;
    double totalTraceTimeMs = 0.0;
};

class MissedTargetStageTraceWriter
{
public:
    bool writeAll(const CoarseMatchReport& report,
                  const CoarseMatchConfig& config,
                  const ImagePyramid& imagePyramid,
                  const TemplatePyramid& templatePyramid,
                  const std::filesystem::path& reportsDir) const;

    bool writeAll(const MissedTargetStageTraceReport& trace,
                  const std::filesystem::path& reportsDir) const;
};

ShapeMatchOverlayData buildMissedTargetStageTraceOverlay(const MissedTargetStageTraceReport& trace);

} // namespace ShapeMatch
