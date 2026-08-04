#include "shape_match/pipeline_v3/ShapeMatchV3Diagnostic.h"

#include "shape_match/pipeline_v3/AngleViewBuilderV3.h"
#include "shape_match/pipeline_v3/CoarseSearchV3.h"
#include "shape_match/pipeline_v3/ResponseMapBuilderV3.h"
#include "shape_match/pipeline_v3/ScoreKernelV3.h"

#include <nlohmann/json.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>

namespace ShapeMatch {
namespace {

double angleError(double a, double b)
{
    return std::abs(radToDeg(wrapToPi(a - b)));
}

int nearestAngleIndex(const std::vector<AngleViewV3>& views, double theta)
{
    int best = 0;
    double bestError = std::numeric_limits<double>::max();
    for (int i = 0; i < static_cast<int>(views.size()); ++i) {
        const double error = angleError(views[static_cast<size_t>(i)].angleRadians, theta);
        if (error < bestError) { bestError = error; best = i; }
    }
    return best;
}

bool insideBounds(const ResponseMapV3& response, const AngleViewV3& view, int x, int y)
{
    return x + view.minDx >= 0 && x + view.maxDx < response.width
        && y + view.minDy >= 0 && y + view.maxDy < response.height;
}

GtLevelProbeV3 probeLevel(const ResponseMapV3& response, const PyramidLevelModelV3& levelModel,
                          const ShapeSearchParametersV3& parameters,
                          const GroundTruthInstance& gt, int level, bool coarse)
{
    GtLevelProbeV3 probe;
    probe.level = level;
    probe.x = static_cast<int>(std::lround(gt.pose.x * levelModel.scale));
    probe.y = static_cast<int>(std::lround(gt.pose.y * levelModel.scale));
    probe.angleIndex = nearestAngleIndex(levelModel.angleViews, gt.pose.theta);
    const AngleViewV3& view = levelModel.angleViews[static_cast<size_t>(probe.angleIndex)];
    probe.sampledAngleDeg = radToDeg(view.angleRadians);
    probe.angleQuantizationErrorDeg = angleError(view.angleRadians, gt.pose.theta);
    probe.totalPoints = static_cast<int>(view.points.size());
    probe.insideValidBounds = insideBounds(response, view, probe.x, probe.y);
    if (coarse && parameters.searchRoi.area() > 0) {
        const cv::Rect scaled(static_cast<int>(std::floor(parameters.searchRoi.x * levelModel.scale)),
                              static_cast<int>(std::floor(parameters.searchRoi.y * levelModel.scale)),
                              static_cast<int>(std::ceil(parameters.searchRoi.width * levelModel.scale)),
                              static_cast<int>(std::ceil(parameters.searchRoi.height * levelModel.scale)));
        probe.insideSearchRoi = scaled.contains(cv::Point(probe.x, probe.y));
    }
    if (!probe.insideValidBounds || !probe.insideSearchRoi || view.points.empty()) return probe;

    const int base = probe.y * response.stride + probe.x;
    std::uint32_t sum = 0;
    const std::uint32_t threshold = static_cast<std::uint32_t>(
        std::floor((1.0f - std::clamp(parameters.minScore, 0.0f, 1.0f)) * 255.0f * view.totalWeight));
    for (size_t stageIndex = 0; stageIndex < view.stages.size(); ++stageIndex) {
        const ScoreStageV3& stage = view.stages[stageIndex];
        for (std::uint32_t i = stage.begin; i < stage.end; ++i) {
            const RotatedPointV3& point = view.points[i];
            ++probe.expectedPerBin[point.orientationBin];
            const std::uint8_t cost = response.binData[point.orientationBin][base + point.linearOffset];
            sum += static_cast<std::uint32_t>(cost) * point.weight;
            probe.matchedPerBin[point.orientationBin] += cost < 255 ? 1 : 0;
        }
    }
    (void)threshold; // Baseline response search performs no staged pruning.
    probe.matchedPoints = std::accumulate(probe.matchedPerBin.begin(), probe.matchedPerBin.end(), 0);
    probe.score = view.totalWeight > 0 ? 1.0 - static_cast<double>(sum) / (255.0 * view.totalWeight) : 0.0;
    return probe;
}

struct DiagnosticBounds { int left; int top; int right; int bottom; };

DiagnosticBounds diagnosticBounds(const ResponseMapV3& response, const AngleViewV3& view,
                                  cv::Rect roi)
{
    const cv::Rect image(0, 0, response.width, response.height);
    if (roi.width <= 0 || roi.height <= 0) roi = image;
    roi &= image;
    return {std::max(roi.x, -view.minDx), std::max(roi.y, -view.minDy),
            std::min(roi.x + roi.width, response.width - view.maxDx),
            std::min(roi.y + roi.height, response.height - view.maxDy)};
}

std::uint32_t diagnosticRawCost(const ResponseMapV3& response, const AngleViewV3& view,
                                int x, int y)
{
    const int base = y * response.stride + x;
    std::uint32_t sum = 0;
    for (const RotatedPointV3& point : view.points)
        sum += static_cast<std::uint32_t>(response.binData[point.orientationBin]
            [base + point.linearOffset]) * point.weight;
    return sum;
}

bool sameCandidate(const MatchCandidateV3& a, const MatchCandidateV3& b)
{
    return a.x == b.x && a.y == b.y && a.angleIndex == b.angleIndex;
}

bool suppressesCandidate(const MatchCandidateV3& kept, const MatchCandidateV3& candidate,
                         const std::vector<AngleViewV3>& views, float positionDistance,
                         float angleDistance)
{
    const float dx = static_cast<float>(candidate.x - kept.x);
    const float dy = static_cast<float>(candidate.y - kept.y);
    float angle = std::abs(views[static_cast<size_t>(candidate.angleIndex)].angleRadians
                           - views[static_cast<size_t>(kept.angleIndex)].angleRadians);
    angle = std::min(angle, static_cast<float>(2.0 * kPi) - angle);
    return dx * dx + dy * dy < positionDistance * positionDistance && angle < angleDistance;
}

void addCoarseCandidateDiagnostics(const ResponseMapV3& response,
                                   const PyramidLevelModelV3& levelModel,
                                   const ShapeSearchParametersV3& parameters,
                                   const std::vector<MatchCandidateV3>& top300,
                                   const std::vector<MatchCandidateV3>& nms40,
                                   std::vector<GtDiagnosticV3>& diagnostics)
{
    cv::Rect coarseRoi;
    if (parameters.searchRoi.area() > 0) {
        coarseRoi = cv::Rect(static_cast<int>(std::floor(parameters.searchRoi.x * levelModel.scale)),
            static_cast<int>(std::floor(parameters.searchRoi.y * levelModel.scale)),
            static_cast<int>(std::ceil(parameters.searchRoi.width * levelModel.scale)),
            static_cast<int>(std::ceil(parameters.searchRoi.height * levelModel.scale)));
    }
    const float positionTolerance = std::max(1.0f, parameters.nmsDistance * levelModel.scale);
    const float angleTolerance = std::max(
        static_cast<float>(3.0 * kPi / kPrecomputedAngleCountV3), parameters.nmsAngleRadians);

    std::vector<MatchCandidateV3> nearbyBest(diagnostics.size());
    std::vector<bool> hasBest(diagnostics.size(), false);
    for (size_t gi = 0; gi < diagnostics.size(); ++gi) {
        GtLevelProbeV3& probe = diagnostics[gi].levels.back();
        const double gtAngle = diagnostics[gi].groundTruth.pose.theta;
        const int radius = static_cast<int>(std::ceil(positionTolerance));
        for (int ai = 0; ai < static_cast<int>(levelModel.angleViews.size()); ++ai) {
            const AngleViewV3& view = levelModel.angleViews[static_cast<size_t>(ai)];
            if (angleError(view.angleRadians, gtAngle) > radToDeg(angleTolerance)) continue;
            const DiagnosticBounds bounds = diagnosticBounds(response, view, coarseRoi);
            for (int y = std::max(bounds.top, probe.y - radius);
                 y < std::min(bounds.bottom, probe.y + radius + 1); ++y) {
                for (int x = std::max(bounds.left, probe.x - radius);
                     x < std::min(bounds.right, probe.x + radius + 1); ++x) {
                    const float dx = static_cast<float>(x - probe.x);
                    const float dy = static_cast<float>(y - probe.y);
                    if (dx * dx + dy * dy > positionTolerance * positionTolerance) continue;
                    const std::uint32_t raw = diagnosticRawCost(response, view, x, y);
                    if (!hasBest[gi] || raw < nearbyBest[gi].rawCost) {
                        hasBest[gi] = true;
                        nearbyBest[gi] = {x, y, ai, probe.level, raw,
                            1.0f - static_cast<float>(raw)
                                / static_cast<float>(255u * view.totalWeight)};
                    }
                }
            }
        }
        if (!hasBest[gi]) continue;
        const MatchCandidateV3& best = nearbyBest[gi];
        probe.hasNearbyBestCandidate = true;
        probe.nearbyBestX = best.x; probe.nearbyBestY = best.y;
        probe.nearbyBestAngleIndex = best.angleIndex;
        probe.nearbyBestAngleDeg = radToDeg(
            levelModel.angleViews[static_cast<size_t>(best.angleIndex)].angleRadians);
        probe.nearbyBestScore = best.score; probe.nearbyBestRawCost = best.rawCost;
        probe.enteredGlobalTop300 = std::any_of(top300.begin(), top300.end(),
            [&](const MatchCandidateV3& candidate) { return sameCandidate(candidate, best); });
        probe.enteredNmsTop40 = std::any_of(nms40.begin(), nms40.end(),
            [&](const MatchCandidateV3& candidate) { return sameCandidate(candidate, best); });
        if (probe.enteredGlobalTop300 && !probe.enteredNmsTop40) {
            for (const MatchCandidateV3& kept : nms40) {
                if (!suppressesCandidate(kept, best, levelModel.angleViews,
                                         positionTolerance, angleTolerance)) continue;
                probe.suppressedByNms = true;
                probe.suppressorX = kept.x; probe.suppressorY = kept.y;
                probe.suppressorAngleIndex = kept.angleIndex;
                probe.suppressorAngleDeg = radToDeg(
                    levelModel.angleViews[static_cast<size_t>(kept.angleIndex)].angleRadians);
                probe.suppressorScore = kept.score;
                break;
            }
            probe.droppedByNmsCapacity = !probe.suppressedByNms;
        }
    }

    std::vector<std::uint64_t> betterCounts(diagnostics.size(), 0);
    for (int ai = 0; ai < static_cast<int>(levelModel.angleViews.size()); ++ai) {
        const AngleViewV3& view = levelModel.angleViews[static_cast<size_t>(ai)];
        const DiagnosticBounds bounds = diagnosticBounds(response, view, coarseRoi);
        for (int y = bounds.top; y < bounds.bottom; ++y) {
            int x = bounds.left;
            for (; x + 32 <= bounds.right; x += 32) {
                const ScoreBlock32V3 block = cpuSupportsAvx2V3()
                    ? scoreBlock32AVX2(response, y * response.stride + x, view,
                                       std::numeric_limits<std::uint32_t>::max())
                    : scoreBlock32Scalar(response, y * response.stride + x, view,
                                         std::numeric_limits<std::uint32_t>::max());
                for (int lane = 0; lane < 32; ++lane)
                    for (size_t gi = 0; gi < diagnostics.size(); ++gi)
                        if (hasBest[gi] && block.rawCosts[static_cast<size_t>(lane)] < nearbyBest[gi].rawCost)
                            ++betterCounts[gi];
            }
            for (; x < bounds.right; ++x) {
                const std::uint32_t raw = diagnosticRawCost(response, view, x, y);
                for (size_t gi = 0; gi < diagnostics.size(); ++gi)
                    if (hasBest[gi] && raw < nearbyBest[gi].rawCost) ++betterCounts[gi];
            }
        }
    }
    for (size_t gi = 0; gi < diagnostics.size(); ++gi)
        if (hasBest[gi]) diagnostics[gi].levels.back().nearbyBestGlobalRank = betterCounts[gi] + 1;
}

std::string diagnose(const GtDiagnosticV3& diagnostic, float minScore)
{
    if (diagnostic.detected) return "detected";
    if (diagnostic.levels.empty()) return "no_pyramid_probe";
    const GtLevelProbeV3& coarse = diagnostic.levels.back();
    if (!coarse.insideSearchRoi) return "gt_origin_outside_search_roi";
    if (!coarse.insideValidBounds) return "template_outside_image_at_coarse_level";
    if (coarse.rejectedStage > 0) return "coarse_stage_" + std::to_string(coarse.rejectedStage) + "_pruned";
    if (coarse.score < minScore) return "coarse_score_below_min_score";
    if (coarse.hasNearbyBestCandidate && !coarse.enteredGlobalTop300)
        return "coarse_nearby_best_ranked_out_before_top300";
    if (coarse.suppressedByNms) return "coarse_nearby_best_suppressed_by_nms";
    if (coarse.droppedByNmsCapacity) return "coarse_nearby_best_dropped_by_nms_top40_capacity";
    if (coarse.enteredNmsTop40) return "coarse_candidate_passed_nms_but_lost_at_level0_or_final_nms";
    for (auto it = diagnostic.levels.rbegin() + 1; it != diagnostic.levels.rend(); ++it) {
        if (!it->insideValidBounds) return "template_outside_image_at_level_" + std::to_string(it->level);
        if (it->rejectedStage > 0) return "level_" + std::to_string(it->level) + "_stage_"
            + std::to_string(it->rejectedStage) + "_pruned";
        if (it->score < minScore) return "level_" + std::to_string(it->level) + "_score_below_min_score";
    }
    return "passed_gt_probe_but_lost_by_topk_tracking_or_nms";
}

} // namespace

ShapeMatchV3DiagnosticReport ShapeMatchV3DiagnosticAnalyzer::analyze(
    const cv::Mat& image, const ShapeModelV3& model, const ShapeSearchParametersV3& p,
    const ShapeMatchStatisticsV3& statistics, const std::vector<MatchResultV3>& results,
    const std::vector<GroundTruthInstance>& groundTruth) const
{
    ShapeMatchV3DiagnosticReport report;
    report.searchParameters = p; report.statistics = statistics; report.results = results;
    report.orientationBinCount = 1;
    report.resultCount = static_cast<int>(results.size()); report.gtCount = static_cast<int>(groundTruth.size());
    if (groundTruth.empty()) return report;
    int levels = p.pyramidLevels > 0 ? p.pyramidLevels : model.parameters.pyramidLevels;
    levels = std::clamp(levels, 1, kPyramidLevelCountV3);
    std::vector<cv::Mat> images {image};
    while (static_cast<int>(images.size()) < levels && images.back().cols >= 32 && images.back().rows >= 32) {
        cv::Mat next; cv::pyrDown(images.back(), next); images.push_back(std::move(next));
    }
    levels = static_cast<int>(images.size());
    const int coarseLevel = levels - 1;
    const float baseStep = static_cast<float>(2.0 * kPi / kPrecomputedAngleCountV3);
    ResponseMapV3 response = ResponseMapBuilderV3().build(
        images[static_cast<size_t>(coarseLevel)], model.parameters.gradientLow, model.parameters.gradientHigh,
        model.parameters.nonMaximumSuppression);
    PyramidLevelModelV3 levelModel;
    levelModel.scale = std::ldexp(1.0f, -coarseLevel);
    levelModel.stride = response.stride;
    levelModel.angleViews = AngleViewBuilderV3().bindPrecomputed(
        model, coarseLevel, levelModel.stride);

    for (const GroundTruthInstance& gt : groundTruth) {
        GtDiagnosticV3 diagnostic; diagnostic.groundTruth = gt;
        double nearestCost = std::numeric_limits<double>::max();
        for (int i = 0; i < static_cast<int>(results.size()); ++i) {
            const double dx = results[static_cast<size_t>(i)].pose.x - gt.pose.x;
            const double dy = results[static_cast<size_t>(i)].pose.y - gt.pose.y;
            const double dp = std::sqrt(dx * dx + dy * dy);
            const double da = angleError(results[static_cast<size_t>(i)].pose.theta, gt.pose.theta);
            const double cost = dp + da;
            if (cost < nearestCost) {
                nearestCost = cost; diagnostic.nearestResultIndex = i;
                diagnostic.positionErrorPx = dp; diagnostic.angleErrorDeg = da;
                diagnostic.nearestResultScore = results[static_cast<size_t>(i)].score;
            }
        }
        diagnostic.levels.push_back(probeLevel(response, levelModel, p, gt, coarseLevel, true));
        report.groundTruth.push_back(std::move(diagnostic));
    }

    ShapeSearchParametersV3 coarseParameters = p;
    if (p.searchRoi.area() > 0) {
        const float scale = levelModel.scale;
        coarseParameters.searchRoi = cv::Rect(
            static_cast<int>(std::floor(p.searchRoi.x * scale)),
            static_cast<int>(std::floor(p.searchRoi.y * scale)),
            static_cast<int>(std::ceil(p.searchRoi.width * scale)),
            static_cast<int>(std::ceil(p.searchRoi.height * scale)));
    }
    std::vector<MatchCandidateV3> top300 = CoarseSearchV3().search(
        response, levelModel, coarseParameters, nullptr);
    CandidateCollectorV3::keepTopK(top300, p.coarseTopK);
    const std::vector<MatchCandidateV3> nms40 = PoseNmsV3().apply(
        top300, levelModel.angleViews, p.maxRefineCandidates,
        std::max(1.0f, p.nmsDistance * levelModel.scale),
        std::max(baseStep * 1.5f, p.nmsAngleRadians));
    addCoarseCandidateDiagnostics(response, levelModel, p, top300, nms40,
                                  report.groundTruth);

    struct Pair { int gt = -1; int result = -1; double cost = 0.0; };
    std::vector<Pair> pairs;
    for (int gi = 0; gi < static_cast<int>(report.groundTruth.size()); ++gi) {
        const MatchPose& gt = report.groundTruth[static_cast<size_t>(gi)].groundTruth.pose;
        for (int ri = 0; ri < static_cast<int>(results.size()); ++ri) {
            const MatchPose& result = results[static_cast<size_t>(ri)].pose;
            const double dx = result.x - gt.x, dy = result.y - gt.y;
            const double dp = std::sqrt(dx * dx + dy * dy);
            const double da = angleError(result.theta, gt.theta);
            if (dp <= p.nmsDistance && da <= radToDeg(p.nmsAngleRadians))
                pairs.push_back({gi, ri, dp / std::max(0.001f, p.nmsDistance)
                    + da / std::max(0.001, radToDeg(p.nmsAngleRadians))});
        }
    }
    std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) { return a.cost < b.cost; });
    std::vector<bool> usedGt(report.groundTruth.size(), false), usedResult(results.size(), false);
    for (const Pair& pair : pairs) {
        if (usedGt[static_cast<size_t>(pair.gt)] || usedResult[static_cast<size_t>(pair.result)]) continue;
        usedGt[static_cast<size_t>(pair.gt)] = true; usedResult[static_cast<size_t>(pair.result)] = true;
        report.groundTruth[static_cast<size_t>(pair.gt)].detected = true;
        ++report.detectedCount;
    }
    for (GtDiagnosticV3& diagnostic : report.groundTruth)
        diagnostic.missedReason = diagnose(diagnostic, p.minScore);
    return report;
}

bool ShapeMatchV3ReportWriter::write(const ShapeMatchV3DiagnosticReport& report,
                                     const std::filesystem::path& dir) const
{
    std::filesystem::create_directories(dir);
    nlohmann::json root;
    root["image"] = report.imageName; root["template"] = report.templateName;
    root["result_count"] = report.resultCount; root["gt_count"] = report.gtCount;
    root["detected_count"] = report.detectedCount;
    root["recall"] = report.gtCount > 0 ? static_cast<double>(report.detectedCount) / report.gtCount : 0.0;
    root["parameters"] = {{"orientation_bins", report.orientationBinCount}, {"min_score", report.searchParameters.minScore},
        {"coarse_top_k", report.searchParameters.coarseTopK}, {"max_matches", report.searchParameters.maxMatches},
        {"pyramid_levels", report.searchParameters.pyramidLevels},
        {"refined_min_visible_ratio", report.searchParameters.refinedMinVisibleRatio},
        {"refined_min_correspondence_ratio", report.searchParameters.refinedMinCorrespondenceRatio},
        {"refined_max_rms_residual", report.searchParameters.refinedMaxRmsResidual},
        {"refined_nms_template_fraction", report.searchParameters.refinedNmsTemplateFraction},
        {"angle_step_deg", radToDeg(report.searchParameters.angleStepRadians)}};
    root["statistics"] = {{"total_ms", report.statistics.totalTimeMs},
        {"response_ms", report.statistics.responseMapTimeMs}, {"coarse_ms", report.statistics.coarseSearchTimeMs},
        {"pyramid_ms", report.statistics.pyramidTimeMs}, {"angle_view_ms", report.statistics.angleViewTimeMs},
        {"track_ms", report.statistics.pyramidTrackTimeMs}, {"refine_ms", report.statistics.refineTimeMs},
        {"verify_ms", report.statistics.verifyTimeMs}, {"evaluated", report.statistics.evaluatedCandidates},
        {"rejected_stage0", report.statistics.rejectedAtStage0},
        {"rejected_stage1", report.statistics.rejectedAtStage1},
        {"rejected_stage2", report.statistics.rejectedAtStage2}};
    for (const MatchResultV3& result : report.results) {
        root["results"].push_back({{"x", result.pose.x}, {"y", result.pose.y},
            {"theta_deg", result.pose.thetaDeg()}, {"score", result.score},
            {"raw_cost", result.rawCost}, {"refined", result.refined},
            {"refinement_iterations", result.refinementIterations},
            {"valid_correspondences", result.validCorrespondences},
            {"visible_ratio", result.visibleRatio}, {"rms_residual", result.rmsResidual}});
    }
    for (const GtDiagnosticV3& gt : report.groundTruth) {
        nlohmann::json item = {{"id", gt.groundTruth.id}, {"pose", {{"x", gt.groundTruth.pose.x},
            {"y", gt.groundTruth.pose.y}, {"theta_deg", gt.groundTruth.pose.thetaDeg()}, {"scale", gt.groundTruth.pose.scale}}},
            {"detected", gt.detected}, {"missed_reason", gt.missedReason},
            {"nearest_result_index", gt.nearestResultIndex}, {"nearest_result_score", gt.nearestResultScore},
            {"position_error_px", gt.positionErrorPx}, {"angle_error_deg", gt.angleErrorDeg}};
        for (const GtLevelProbeV3& level : gt.levels) {
            nlohmann::json levelItem = {{"level", level.level}, {"x", level.x}, {"y", level.y},
                {"angle_index", level.angleIndex}, {"sampled_angle_deg", level.sampledAngleDeg},
                {"angle_quantization_error_deg", level.angleQuantizationErrorDeg},
                {"matched_points", level.matchedPoints}, {"total_points", level.totalPoints}, {"score", level.score},
                {"rejected_stage", level.rejectedStage}, {"inside_search_roi", level.insideSearchRoi},
                {"inside_valid_bounds", level.insideValidBounds}, {"expected_per_bin", level.expectedPerBin},
                {"matched_per_bin", level.matchedPerBin}};
            levelItem["coarse_candidate_stages"] = {
                {"nearby_best", {{"found", level.hasNearbyBestCandidate},
                    {"x", level.nearbyBestX}, {"y", level.nearbyBestY},
                    {"angle_index", level.nearbyBestAngleIndex},
                    {"angle_deg", level.nearbyBestAngleDeg}, {"score", level.nearbyBestScore},
                    {"raw_cost", level.nearbyBestRawCost},
                    {"global_rank", level.nearbyBestGlobalRank}}},
                {"entered_top300", level.enteredGlobalTop300},
                {"entered_nms_top40", level.enteredNmsTop40},
                {"nms_suppressed", level.suppressedByNms},
                {"nms_capacity_dropped", level.droppedByNmsCapacity},
                {"suppressor", {{"x", level.suppressorX}, {"y", level.suppressorY},
                    {"angle_index", level.suppressorAngleIndex},
                    {"angle_deg", level.suppressorAngleDeg}, {"score", level.suppressorScore}}}};
            item["levels"].push_back(std::move(levelItem));
        }
        root["ground_truth"].push_back(std::move(item));
    }
    std::ofstream json(dir / "latest_shape_match_v3_diagnostic.json");
    if (!json) return false; json << root.dump(2); json.close();
    std::ofstream csv(dir / "latest_shape_match_v3_gt.csv");
    if (!csv) return false;
    csv << "id,detected,reason,gt_x,gt_y,gt_theta_deg,nearest_result,position_error_px,angle_error_deg,level,probe_x,probe_y,sampled_angle_deg,angle_quant_error_deg,matched_points,total_points,score,rejected_stage,inside_roi,inside_bounds,nearby_best_found,nearby_best_x,nearby_best_y,nearby_best_angle_deg,nearby_best_score,nearby_best_raw_cost,nearby_best_global_rank,entered_top300,entered_nms_top40,nms_suppressed,nms_capacity_dropped,suppressor_x,suppressor_y,suppressor_angle_deg,suppressor_score\n";
    csv << std::fixed << std::setprecision(6);
    for (const GtDiagnosticV3& gt : report.groundTruth)
        for (const GtLevelProbeV3& level : gt.levels)
            csv << gt.groundTruth.id << ',' << (gt.detected ? "true" : "false") << ',' << gt.missedReason << ','
                << gt.groundTruth.pose.x << ',' << gt.groundTruth.pose.y << ',' << gt.groundTruth.pose.thetaDeg() << ','
                << gt.nearestResultIndex << ',' << gt.positionErrorPx << ',' << gt.angleErrorDeg << ',' << level.level << ','
                << level.x << ',' << level.y << ',' << level.sampledAngleDeg << ',' << level.angleQuantizationErrorDeg << ','
                << level.matchedPoints << ',' << level.totalPoints << ',' << level.score << ',' << level.rejectedStage << ','
                << (level.insideSearchRoi ? "true" : "false") << ',' << (level.insideValidBounds ? "true" : "false") << ','
                << (level.hasNearbyBestCandidate ? "true" : "false") << ',' << level.nearbyBestX << ','
                << level.nearbyBestY << ',' << level.nearbyBestAngleDeg << ',' << level.nearbyBestScore << ','
                << level.nearbyBestRawCost << ',' << level.nearbyBestGlobalRank << ','
                << (level.enteredGlobalTop300 ? "true" : "false") << ','
                << (level.enteredNmsTop40 ? "true" : "false") << ','
                << (level.suppressedByNms ? "true" : "false") << ','
                << (level.droppedByNmsCapacity ? "true" : "false") << ','
                << level.suppressorX << ',' << level.suppressorY << ',' << level.suppressorAngleDeg << ','
                << level.suppressorScore << '\n';
    csv.close();
    std::ofstream summary(dir / "latest_shape_match_v3_summary.txt");
    if (!summary) return false;
    summary << "ShapeMatch V3 GT diagnostic\nresults: " << report.resultCount << "\ngt: " << report.gtCount
            << "\ndetected: " << report.detectedCount << "\nmissed: " << report.gtCount - report.detectedCount << "\n";
    for (const GtDiagnosticV3& gt : report.groundTruth)
        summary << gt.groundTruth.id << ": " << (gt.detected ? "detected" : "MISSED")
                << " reason=" << gt.missedReason << " dxy=" << gt.positionErrorPx
                << " dtheta_deg=" << gt.angleErrorDeg << '\n';
    return summary.good();
}

} // namespace ShapeMatch
