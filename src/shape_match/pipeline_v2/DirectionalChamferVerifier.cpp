#include "shape_match/pipeline_v2/DirectionalChamferVerifier.h"

#include "shape_match/core/ShapeMatchTypes.h"

#include <algorithm>
#include <cmath>

namespace ShapeMatch {

namespace {

int wrapBin(int bin, int count)
{
    if (count <= 0) {
        return 0;
    }
    bin %= count;
    return bin < 0 ? bin + count : bin;
}

int binFromVector(double x, double y, int count)
{
    const double angle = std::atan2(y, x);
    return wrapBin(static_cast<int>(std::floor((angle + kPi) / (2.0 * kPi) * count)), count);
}

double gaussian(double x, double sigma)
{
    sigma = std::max(1e-6, sigma);
    return std::exp(-(x * x) / (2.0 * sigma * sigma));
}

} // namespace

DirectionalChamferVerifier::DirectionalChamferVerifier(DirectionalChamferConfig config)
    : m_config(std::move(config))
{
}

DirectionalChamferScore DirectionalChamferVerifier::scoreCandidate(const MatchPose& pose,
                                                                   const SegmentTemplate& templ,
                                                                   const DirectionalDistanceField& field,
                                                                   double pruningFloor) const
{
    DirectionalChamferScore score;
    score.totalPoints = templ.pointCount();
    score.totalSegments = templ.segmentCount();
    if (templ.empty() || !field.valid || field.distanceBins.empty()) {
        score.rejectReason = "invalid_directional_chamfer_input";
        return score;
    }

    const double c = std::cos(pose.theta);
    const double s = std::sin(pose.theta);
    double totalDistanceScore = 0.0;
    double totalOrientationScore = 0.0;
    double totalWeight = 0.0;
    int evaluatedGlobal = 0;
    int matchedGlobal = 0;

    score.segments.reserve(templ.segments.size());
    for (const SegmentTemplate::SegmentInfo& seg : templ.segments) {
        SegmentScore segScore;
        segScore.segmentId = seg.segmentId;
        segScore.totalPoints = seg.count;
        double segDistanceScore = 0.0;
        double segOrientationScore = 0.0;
        double segDistanceSum = 0.0;
        double segOrientationSum = 0.0;
        double segWeight = 0.0;

        for (int i = 0; i < seg.count; ++i) {
            const SegmentVerifyPoint& p = templ.points[static_cast<size_t>(seg.start + i)];
            const double x = pose.x + pose.scale * (c * p.x - s * p.y);
            const double y = pose.y + pose.scale * (s * p.x + c * p.y);
            const int ix = static_cast<int>(std::round(x));
            const int iy = static_cast<int>(std::round(y));
            ++score.evaluatedPoints;
            ++segScore.evaluatedPoints;
            ++evaluatedGlobal;
            if (ix < 0 || iy < 0 || ix >= field.imageSize.width || iy >= field.imageSize.height) {
                continue;
            }
            const double rgx = c * p.gx - s * p.gy;
            const double rgy = s * p.gx + c * p.gy;
            const int bin = binFromVector(rgx, rgy, field.orientationBinCount);
            const float d = field.distanceBins[static_cast<size_t>(bin)].at<float>(iy, ix);
            const double clampedDistance = std::min<double>(d, m_config.maxDistancePx);
            const double ds = gaussian(clampedDistance, m_config.distanceSigmaPx);
            const double os = d <= m_config.maxDistancePx ? 1.0 : gaussian(m_config.maxOrientationErrorDeg, m_config.orientationSigmaDeg);
            const double w = std::max(0.001f, p.weight);
            segDistanceScore += ds * w;
            segOrientationScore += os * w;
            segDistanceSum += clampedDistance;
            segOrientationSum += (1.0 - os) * m_config.maxOrientationErrorDeg;
            segWeight += w;
            totalWeight += w;
            totalDistanceScore += ds * w;
            totalOrientationScore += os * w;
            if (d <= m_config.maxDistancePx) {
                ++segScore.matchedPoints;
                ++matchedGlobal;
            }

            if (m_config.enableGreedyPruning &&
                pruningFloor > 0.0 &&
                evaluatedGlobal >= m_config.minEvaluatedPointsBeforePruning &&
                evaluatedGlobal % std::max(1, m_config.pruningCheckInterval) == 0) {
                const int remaining = std::max(0, score.totalPoints - evaluatedGlobal);
                const double currentBest = totalWeight > 0.0 ? totalDistanceScore / totalWeight : 0.0;
                const double upper = (currentBest * evaluatedGlobal + remaining) /
                                     static_cast<double>(std::max(1, score.totalPoints));
                if (upper + m_config.upperBoundMargin < pruningFloor) {
                    score.pruned = true;
                    score.rejectReason = "directional_chamfer_upper_bound_pruned";
                    break;
                }
            }
        }
        if (segWeight > 0.0) {
            segScore.distanceScore = segDistanceScore / segWeight;
            segScore.orientationScore = segOrientationScore / segWeight;
            segScore.meanDistance = segScore.evaluatedPoints > 0 ? segDistanceSum / segScore.evaluatedPoints : 0.0;
            segScore.meanOrientationErrorDeg = segScore.evaluatedPoints > 0 ? segOrientationSum / segScore.evaluatedPoints : 0.0;
        }
        segScore.coverage = segScore.totalPoints > 0 ? static_cast<double>(segScore.matchedPoints) / segScore.totalPoints : 0.0;
        segScore.segmentScore = m_config.wDistance * segScore.distanceScore +
                                m_config.wOrientation * segScore.orientationScore +
                                m_config.wCoverage * segScore.coverage;
        segScore.active = segScore.segmentScore >= m_config.minSegmentScore &&
                          segScore.coverage >= m_config.minSegmentCoverage;
        if (segScore.active) {
            ++score.activeSegments;
        }
        score.segments.push_back(segScore);
        if (score.pruned) {
            break;
        }
    }

    score.matchedPoints = matchedGlobal;
    if (totalWeight > 0.0) {
        score.distanceScore = totalDistanceScore / totalWeight;
        score.orientationScore = totalOrientationScore / totalWeight;
    }
    score.coverageScore = score.totalPoints > 0 ? static_cast<double>(score.matchedPoints) / score.totalPoints : 0.0;
    score.segmentCoverageScore = score.totalSegments > 0 ? static_cast<double>(score.activeSegments) / score.totalSegments : 0.0;
    const double coverageTerm = m_config.enableSegmentScoring ? score.segmentCoverageScore : score.coverageScore;
    score.finalScore = m_config.wDistance * score.distanceScore +
                       m_config.wOrientation * score.orientationScore +
                       m_config.wCoverage * coverageTerm;
    score.accepted = !score.pruned &&
                     score.coverageScore >= m_config.minTotalCoverage &&
                     (!m_config.enableSegmentScoring || score.activeSegments > 0);
    if (!score.accepted && score.rejectReason.empty()) {
        score.rejectReason = score.coverageScore < m_config.minTotalCoverage ? "directional_chamfer_low_coverage"
                                                                             : "directional_chamfer_no_active_segment";
    }
    return score;
}

std::vector<VerifiedResponsePeak> DirectionalChamferVerifier::verifyPeaks(const std::vector<VerifiedResponsePeak>& input,
                                                                          const SegmentTemplate& templ,
                                                                          const DirectionalDistanceField& field,
                                                                          std::vector<DirectionalChamferScore>* scores,
                                                                          int* prunedCount) const
{
    std::vector<VerifiedResponsePeak> output;
    if (scores != nullptr) {
        scores->clear();
    }
    if (prunedCount != nullptr) {
        *prunedCount = 0;
    }
    const int count = std::min<int>(static_cast<int>(input.size()), std::max(1, m_config.maxCandidatesToVerify));
    output.reserve(static_cast<size_t>(count));
    double pruningFloor = -1.0;
    for (int i = 0; i < count; ++i) {
        VerifiedResponsePeak peak = input[static_cast<size_t>(i)];
        DirectionalChamferScore score = scoreCandidate(peak.pose, templ, field, pruningFloor);
        peak.combinedScore = score.finalScore;
        peak.accepted = score.accepted;
        peak.rejectReason = score.rejectReason;
        if (score.accepted) {
            output.push_back(peak);
            std::sort(output.begin(), output.end(), [](const VerifiedResponsePeak& a, const VerifiedResponsePeak& b) {
                return a.combinedScore > b.combinedScore;
            });
            if (static_cast<int>(output.size()) > m_config.targetCandidatesAfterVerify) {
                output.resize(static_cast<size_t>(m_config.targetCandidatesAfterVerify));
            }
            if (static_cast<int>(output.size()) >= m_config.targetCandidatesAfterVerify) {
                pruningFloor = output.back().combinedScore;
            }
        }
        if (score.pruned && prunedCount != nullptr) {
            ++(*prunedCount);
        }
        if (scores != nullptr) {
            scores->push_back(score);
        }
    }
    return output;
}

} // namespace ShapeMatch
