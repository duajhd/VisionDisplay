#pragma once

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace ShapeMatch {

constexpr double kPi = 3.14159265358979323846;

enum class EdgePolarity {
    Any = 0,
    DarkToBright,
    BrightToDark
};

struct MatchPose
{
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double scale = 1.0;

    double thetaDeg() const;
    static MatchPose fromDeg(double x, double y, double thetaDeg, double scale = 1.0);
    cv::Point2d transformPoint(const cv::Point2d& p) const;
    cv::Point2d rotateVector(const cv::Point2d& v) const;
};

struct TemplatePoint
{
    int id = -1;
    cv::Point2d position;
    cv::Point2d normal;
    cv::Point2d tangent;
    cv::Point2d gradientDir;
    double gradientMag = 1.0;
    double weight = 1.0;
    int chainId = 0;
    EdgePolarity polarity = EdgePolarity::Any;
};

struct PointEval
{
    int templatePointId = -1;
    cv::Point2d templatePt;
    cv::Point2d predictedPt;
    cv::Point2d matchedEdgePt;
    bool valid = false;
    bool found = false;
    bool inlier = false;
    bool polarityOk = false;
    double distance = 0.0;
    double normalResidual = 0.0;
    double tangentResidual = 0.0;
    double angleDiffRad = 0.0;
    double angleDiffDeg = 0.0;
    double gradientMag = 0.0;
    double weight = 1.0;
    double distanceScore = 0.0;
    double orientationScore = 0.0;
    double polarityScore = 0.0;
    double pointScore = 0.0;
    std::string rejectReason;
};

struct MatchScore
{
    double finalScore = 0.0;
    double coverageScore = 0.0;
    double distanceScore = 0.0;
    double orientationScore = 0.0;
    double polarityScore = 0.0;
    double continuityScore = 0.0;
    double distributionScore = 0.0;
    double stabilityScore = 0.0;
    double ambiguityPenalty = 0.0;
    double occlusionPenalty = 0.0;
    int totalPoints = 0;
    int validPoints = 0;
    int foundPoints = 0;
    int inlierPoints = 0;
    int polarityOkPoints = 0;
    double coverageRatio = 0.0;
    double inlierRatio = 0.0;
    double meanError = 0.0;
    double rmsError = 0.0;
    double medianError = 0.0;
    double p75Error = 0.0;
    double p90Error = 0.0;
    double p95Error = 0.0;
    double maxError = 0.0;
    double meanAngleDiffDeg = 0.0;
    double medianAngleDiffDeg = 0.0;
    double p90AngleDiffDeg = 0.0;
    bool accepted = false;
    std::string rejectReason;
};

struct PoseError
{
    bool hasGroundTruth = false;
    double dx = 0.0;
    double dy = 0.0;
    double dxy = 0.0;
    double dthetaRad = 0.0;
    double dthetaDeg = 0.0;
    double dscale = 0.0;
    bool xyOk = false;
    bool angleOk = false;
    bool scaleOk = false;
    bool poseOk = false;
};

struct ContourReprojectionError
{
    double meanError = 0.0;
    double rmsError = 0.0;
    double medianError = 0.0;
    double p90Error = 0.0;
    double maxError = 0.0;
    int pointCount = 0;
};

struct ScoredCandidate
{
    int rank = -1;
    MatchPose pose;
    MatchScore score;
    PoseError poseError;
    ContourReprojectionError contourError;
    std::vector<PointEval> pointEvaluations;
};

struct MatchEvaluationStats
{
    int distanceFieldQueryCount = 0;
    int roiDistanceFieldQueryCount = 0;
    int fullDistanceFieldQueryCount = 0;
    int localWindowFallbackCount = 0;
    int linearScanFallbackCount = 0;
    int missingFieldCount = 0;
    int failedLookupCount = 0;
    std::string evaluationMode = "unknown";
};

struct FinalRankerProfile
{
    double totalFinalRankerTimeMs = 0.0;
    int candidateCountBeforeNms = 0;
    int candidateCountAfterNms = 0;
    int candidateCountActuallyEvaluated = 0;
    std::string evaluationMode = "unknown";
    bool pointEvalEnabled = false;
    int pointEvalTopK = 0;
    bool distanceFieldValid = false;
    int roiDistanceFieldQueryCount = 0;
    int fullDistanceFieldQueryCount = 0;
    int linearScanFallbackCount = 0;
    int localWindowFallbackCount = 0;
    int missingFieldCount = 0;
    int numThreads = 1;
    double scoreOnlyTimeMs = 0.0;
    double pointEvalTimeMs = 0.0;
    double avgTimePerCandidateMs = 0.0;
    double avgTimePerTemplatePointUs = 0.0;
};

struct GroundTruthInstance
{
    std::string id;
    MatchPose pose;
};

struct MultiTargetEvalResult
{
    int candidateCount = 0;
    int gtCount = 0;
    int truePositive = 0;
    int falsePositive = 0;
    int falseNegative = 0;
    int duplicateCount = 0;
    double precision = 0.0;
    double recall = 0.0;
    double f1 = 0.0;
    bool top1Hit = false;
    bool top3Hit = false;
    bool top5Hit = false;
    bool top10Hit = false;
    std::vector<int> matchedCandidateIndices;
    std::vector<int> matchedGtIndices;
};

struct MatchDiagnosticReport
{
    std::string imageName;
    std::string templateName;
    int candidateCount = 0;
    std::vector<ScoredCandidate> topCandidates;
    std::vector<GroundTruthInstance> groundTruthInstances;
    MultiTargetEvalResult multiTargetEval;
    double coarseTimeMs = 0.0;
    double refineTimeMs = 0.0;
    double evaluationTimeMs = 0.0;
    double totalTimeMs = 0.0;
    int pyramidLevel = -1;
    std::string failureReason;
};

double radToDeg(double radians);
double degToRad(double degrees);
double wrapToPi(double radians);
double norm(const cv::Point2d& v);
cv::Point2d normalized(const cv::Point2d& v, const cv::Point2d& fallback = cv::Point2d(1.0, 0.0));
double angleBetweenUnitVectors(const cv::Point2d& a, const cv::Point2d& b);
std::string polarityToString(EdgePolarity polarity);

} // namespace ShapeMatch
