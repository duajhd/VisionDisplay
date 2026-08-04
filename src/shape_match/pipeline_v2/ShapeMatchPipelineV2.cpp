#include "shape_match/pipeline_v2/ShapeMatchPipelineV2.h"

#include "shape_match/coarse/ResponseCandidateGenerator.h"
#include "shape_match/coarse/FastPoseScorer.h"
#include "shape_match/evaluation/CandidateRanker.h"
#include "shape_match/evaluation/MultiTargetEvaluator.h"
#include "shape_match/io/DirectionalChamferReportWriter.h"
#include "shape_match/pipeline_v2/DirectionalChamferVerifier.h"
#include "shape_match/pipeline_v2/DirectionalFieldCache.h"
#include "shape_match/pipeline_v2/DirectionalDistanceFieldBuilder.h"
#include "shape_match/pipeline_v2/ParallelDirectionalChamferVerifier.h"
#include "shape_match/pipeline_v2/ResponsePeakCandidateSelector.h"
#include "shape_match/pipeline_v2/ResponsePeakRegionGenerator.h"
#include "shape_match/pipeline_v2/ResponsePeakVerifier.h"
#include "shape_match/pipeline_v2/SegmentTemplate.h"
#include "shape_match/pipeline_v2/TemplateRuntimeCache.h"
#include "shape_match/pipeline_v2/IShapeCandidateGenerator.h"
#include "shape_match/pipeline_v2/OldVotingCandidateGeneratorAdapter.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <utility>

namespace ShapeMatch {

namespace {

double elapsedMs(std::chrono::steady_clock::time_point start)
{
    const auto end = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(end - start).count();
}

bool hasLevel(const ImagePyramid* pyramid, int level)
{
    return pyramid != nullptr && level >= 0 && level < pyramid->levelCount();
}

bool hasLevel(const TemplatePyramid* pyramid, int level)
{
    return pyramid != nullptr && level >= 0 && level < pyramid->levelCount();
}

double levelScale(const ShapeMatchPipelineV2Context& context, int level)
{
    if (context.imagePyramid != nullptr && level >= 0 && level < context.imagePyramid->levelCount()) {
        return context.imagePyramid->scaleOfLevel(level);
    }
    if (context.templatePyramid != nullptr && level >= 0 && level < context.templatePyramid->levelCount()) {
        return context.templatePyramid->scaleOfLevel(level);
    }
    return 1.0;
}

MatchPose scaleGroundTruthPose(const MatchPose& pose, double scale)
{
    MatchPose scaled = pose;
    scaled.x *= scale;
    scaled.y *= scale;
    return scaled;
}

double poseDxy(const MatchPose& a, const MatchPose& b)
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

double poseDthetaDeg(const MatchPose& a, const MatchPose& b)
{
    return std::abs(radToDeg(wrapToPi(a.theta - b.theta)));
}

bool isHit(const MatchPose& candidate, const MatchPose& gt)
{
    return poseDxy(candidate, gt) <= 12.0 && poseDthetaDeg(candidate, gt) <= 12.0;
}

bool topKHit(const std::vector<MatchPose>& candidates,
             const std::vector<GroundTruthInstance>& groundTruth,
             int k,
             double scale)
{
    const int count = std::min<int>(k, static_cast<int>(candidates.size()));
    for (int i = 0; i < count; ++i) {
        for (const GroundTruthInstance& gt : groundTruth) {
            if (isHit(candidates[i], scaleGroundTruthPose(gt.pose, scale))) {
                return true;
            }
        }
    }
    return false;
}

double recallAtAllCandidates(const std::vector<MatchPose>& candidates,
                             const std::vector<GroundTruthInstance>& groundTruth,
                             double scale)
{
    if (groundTruth.empty()) {
        return 0.0;
    }
    int hitCount = 0;
    for (const GroundTruthInstance& gt : groundTruth) {
        const MatchPose scaledGt = scaleGroundTruthPose(gt.pose, scale);
        const bool hit = std::any_of(candidates.begin(), candidates.end(), [&](const MatchPose& candidate) {
            return isHit(candidate, scaledGt);
        });
        if (hit) {
            ++hitCount;
        }
    }
    return static_cast<double>(hitCount) / static_cast<double>(groundTruth.size());
}

void writeTextSummary(const ShapeMatchPipelineV2Config& config,
                      const ShapeMatchPipelineV2Context& context,
                      const ShapeMatchPipelineV2Result& result)
{
    std::filesystem::create_directories(config.reportDir);
    const std::filesystem::path path = config.reportDir / "latest_pipeline_v2_summary.txt";
    std::ofstream out(path);
    if (!out.is_open()) {
        return;
    }

    out << "ShapeMatchPipelineV2 Summary\n";
    out << "mode: " << candidateGenerationModeName(result.mode) << '\n';
    out << "imageName: " << context.imageName << '\n';
    out << "templateName: " << context.templateName << '\n';
    out << "pyramidLevel: " << config.pyramidLevel << '\n';
    out << "executionMode: " << pipelineV2ExecutionModeName(config.responsePipeline.executionMode) << '\n';
    out << "runtimeMode: " << shapeMatchRuntimeModeName(config.production.runtimeMode) << '\n';
    out << "responsePyramidLevel: " << config.responsePipeline.responsePyramidLevel << '\n';
    out << "ok: " << (result.ok ? "true" : "false") << '\n';
    out << "candidateCount: " << result.candidateCount << '\n';
    out << "responsePeakCount: " << result.responsePeakCount << '\n';
    out << "verifiedResponsePeakCount: " << result.verifiedResponsePeakCount << '\n';
    out << "selectedCandidateCount: " << result.selectedCandidateCount << '\n';
    out << "candidateRegionCount: " << result.candidateRegionCount << '\n';
    out << "usedOldLocalRefine: " << (result.usedOldLocalRefine ? "true" : "false") << '\n';
    out << "usedOldBeamPropagation: " << (result.usedOldBeamPropagation ? "true" : "false") << '\n';
    out << "candidateGenerationMs: " << result.candidateGenerationMs << '\n';
    out << "responseGenerationMs: " << result.responseGenerationMs << '\n';
    out << "responseVerificationMs: " << result.responseVerificationMs << '\n';
    out << "candidateSelectionMs: " << result.candidateSelectionMs << '\n';
    out << "regionGenerationMs: " << result.regionGenerationMs << '\n';
    out << "directionalChamferFieldBuildTimeMs: " << result.directionalChamferFieldBuildTimeMs << '\n';
    out << "directionalChamferTimeMs: " << result.directionalChamferTimeMs << '\n';
    out << "directionalChamferInputCount: " << result.directionalChamferInputCount << '\n';
    out << "directionalChamferAcceptedCount: " << result.directionalChamferAcceptedCount << '\n';
    out << "directionalChamferPrunedCount: " << result.directionalChamferPrunedCount << '\n';
    out << "microAdjustTotalMs: " << result.microAdjustTotalMs << '\n';
    out << "microAdjustGenerateMs: " << result.microAdjustGenerateMs << '\n';
    out << "microAdjustScoreMs: " << result.microAdjustScoreMs << '\n';
    out << "microAdjustNmsMs: " << result.microAdjustNmsMs << '\n';
    out << "microAdjustSetupMs: " << result.microAdjustSetupMs << '\n';
    out << "microAdjustSelectedCount: " << result.microAdjustSelectedCount << '\n';
    out << "microAdjustGeneratedCount: " << result.microAdjustGeneratedCount << '\n';
    out << "microAdjustScoredCount: " << result.microAdjustScoredCount << '\n';
    out << "microAdjustInputCandidateCount: " << result.microAdjustInputCandidateCount << '\n';
    out << "finalRankerMs: " << result.finalRankerMs << '\n';
    out << "reportWriteMs: " << result.reportWriteMs << '\n';
    out << "overlayBuildMs: " << result.overlayBuildMs << '\n';
    out << "unaccountedTimeMs: " << result.unaccountedTimeMs << '\n';
    out << "totalTimeMs: " << result.totalTimeMs << '\n';
    out << "gtEvaluated: " << (result.gtEvaluated ? "true" : "false") << '\n';
    out << "recall: " << result.recall << '\n';
    out << "precision: " << result.precision << '\n';
    out << "f1: " << result.f1 << '\n';
    out << "top1Hit: " << (result.top1Hit ? "true" : "false") << '\n';
    out << "top3Hit: " << (result.top3Hit ? "true" : "false") << '\n';
    out << "top5Hit: " << (result.top5Hit ? "true" : "false") << '\n';
    out << "top10Hit: " << (result.top10Hit ? "true" : "false") << '\n';
    out << "failureReason: " << result.failureReason << '\n';
}

void writeProductionSummary(const ShapeMatchPipelineV2Config& config,
                            const ShapeMatchPipelineV2Context& context,
                            const ShapeMatchPipelineV2Result& result)
{
    std::filesystem::create_directories(config.reportDir);
    std::ofstream out(config.reportDir / "latest_pipeline_v2_production_summary.txt");
    if (!out.is_open()) {
        return;
    }
    out << "PipelineV2 Production Summary\n";
    out << "runtimeMode: " << shapeMatchRuntimeModeName(config.production.runtimeMode) << '\n';
    out << "imageName: " << context.imageName << '\n';
    out << "templateName: " << context.templateName << '\n';
    out << "totalTimeMs: " << result.totalTimeMs << '\n';
    out << "responseGenerationMs: " << result.responseGenerationMs << '\n';
    out << "directionalChamferTimeMs: " << result.directionalChamferTimeMs << '\n';
    out << "microAdjustTotalMs: " << result.microAdjustTotalMs << '\n';
    out << "finalRankerMs: " << result.finalRankerMs << '\n';
    out << "reportWriteMs: " << result.reportWriteMs << '\n';
    out << "unaccountedTimeMs: " << result.unaccountedTimeMs << '\n';
    out << "selectedCandidateCount: " << result.selectedCandidateCount << '\n';
    out << "recall: " << result.recall << '\n';
    out << "usedOldLocalRefine: " << (result.usedOldLocalRefine ? "true" : "false") << '\n';
    out << "level0RawChildren: " << result.oldLevel0RawChildren << '\n';
    out << "responseTemplateCacheHits: " << result.responseTemplateCacheHits << '\n';
    out << "segmentTemplateCacheHits: " << result.segmentTemplateCacheHits << '\n';
    out << "directionalFieldCacheHits: " << result.directionalFieldCacheHits << '\n';
    out << "matAllocated: " << result.matAllocatedCount << '\n';
    out << "matReused: " << result.matReusedCount << '\n';
}

void writeResponseSummary(const ShapeMatchPipelineV2Config& config,
                          const ShapeMatchPipelineV2Context& context,
                          const ShapeMatchPipelineV2Result& result)
{
    std::filesystem::create_directories(config.reportDir);
    std::ofstream out(config.reportDir / "latest_pipeline_v2_response_summary.txt");
    if (!out.is_open()) {
        return;
    }
    out << "PipelineV2 Response Summary\n";
    out << "mode: " << candidateGenerationModeName(result.mode) << '\n';
    out << "executionMode: " << pipelineV2ExecutionModeName(config.responsePipeline.executionMode) << '\n';
    out << "imageName: " << context.imageName << '\n';
    out << "templateName: " << context.templateName << '\n';
    out << "responsePyramidLevel: " << config.responsePipeline.responsePyramidLevel << '\n';
    out << "responsePeakCount: " << result.responsePeakCount << '\n';
    out << "verifiedResponsePeakCount: " << result.verifiedResponsePeakCount << '\n';
    out << "selectedCandidateCount: " << result.selectedCandidateCount << '\n';
    out << "candidateRegionCount: " << result.candidateRegionCount << '\n';
    out << "usedOldLocalRefine: " << (result.usedOldLocalRefine ? "true" : "false") << '\n';
    out << "usedOldBeamPropagation: " << (result.usedOldBeamPropagation ? "true" : "false") << '\n';
    out << "oldLevel0RawChildren: " << result.oldLevel0RawChildren << '\n';
    out << "oldLevel0ScoredChildren: " << result.oldLevel0ScoredChildren << '\n';
    out << "responseGenerationMs: " << result.responseGenerationMs << '\n';
    out << "responseVerificationMs: " << result.responseVerificationMs << '\n';
    out << "candidateSelectionMs: " << result.candidateSelectionMs << '\n';
    out << "regionGenerationMs: " << result.regionGenerationMs << '\n';
    out << "roiDistanceFieldBuildMs: " << result.roiDistanceFieldBuildMs << '\n';
    out << "directionalChamferFieldBuildTimeMs: " << result.directionalChamferFieldBuildTimeMs << '\n';
    out << "directionalChamferTimeMs: " << result.directionalChamferTimeMs << '\n';
    out << "directionalChamferInputCount: " << result.directionalChamferInputCount << '\n';
    out << "directionalChamferAcceptedCount: " << result.directionalChamferAcceptedCount << '\n';
    out << "directionalChamferPrunedCount: " << result.directionalChamferPrunedCount << '\n';
    out << "microAdjustTotalMs: " << result.microAdjustTotalMs << '\n';
    out << "microAdjustGenerateMs: " << result.microAdjustGenerateMs << '\n';
    out << "microAdjustScoreMs: " << result.microAdjustScoreMs << '\n';
    out << "microAdjustNmsMs: " << result.microAdjustNmsMs << '\n';
    out << "microAdjustSetupMs: " << result.microAdjustSetupMs << '\n';
    out << "microAdjustSelectedCount: " << result.microAdjustSelectedCount << '\n';
    out << "microAdjustGeneratedCount: " << result.microAdjustGeneratedCount << '\n';
    out << "microAdjustScoredCount: " << result.microAdjustScoredCount << '\n';
    out << "finalRankerMs: " << result.finalRankerMs << '\n';
    out << "reportWriteMs: " << result.reportWriteMs << '\n';
    out << "unaccountedTimeMs: " << result.unaccountedTimeMs << '\n';
    out << "totalTimeMs: " << result.totalTimeMs << '\n';
    out << "recall: " << result.recall << '\n';
    out << "precision: " << result.precision << '\n';
    out << "f1: " << result.f1 << '\n';
    out << "top1Hit: " << (result.top1Hit ? "true" : "false") << '\n';
    out << "top3Hit: " << (result.top3Hit ? "true" : "false") << '\n';
    out << "top5Hit: " << (result.top5Hit ? "true" : "false") << '\n';
    out << "top10Hit: " << (result.top10Hit ? "true" : "false") << '\n';
    out << "failureReason: " << result.failureReason << '\n';
}

void writeResponseProfileJson(const ShapeMatchPipelineV2Config& config, const ShapeMatchPipelineV2Result& result)
{
    std::filesystem::create_directories(config.reportDir);
    nlohmann::json j;
    j["mode"] = candidateGenerationModeName(result.mode);
    j["execution_mode"] = pipelineV2ExecutionModeName(config.responsePipeline.executionMode);
    j["response_pyramid_level"] = config.responsePipeline.responsePyramidLevel;
    j["response_peak_count"] = result.responsePeakCount;
    j["verified_response_peak_count"] = result.verifiedResponsePeakCount;
    j["selected_candidate_count"] = result.selectedCandidateCount;
    j["candidate_region_count"] = result.candidateRegionCount;
    j["used_old_local_refine"] = result.usedOldLocalRefine;
    j["used_old_beam_propagation"] = result.usedOldBeamPropagation;
    j["old_level0_raw_children"] = result.oldLevel0RawChildren;
    j["old_level0_scored_children"] = result.oldLevel0ScoredChildren;
    j["response_generation_ms"] = result.responseGenerationMs;
    j["response_verification_ms"] = result.responseVerificationMs;
    j["candidate_selection_ms"] = result.candidateSelectionMs;
    j["region_generation_ms"] = result.regionGenerationMs;
    j["roi_distance_field_build_ms"] = result.roiDistanceFieldBuildMs;
    j["directional_chamfer_field_build_time_ms"] = result.directionalChamferFieldBuildTimeMs;
    j["directional_chamfer_time_ms"] = result.directionalChamferTimeMs;
    j["directional_chamfer_input_count"] = result.directionalChamferInputCount;
    j["directional_chamfer_accepted_count"] = result.directionalChamferAcceptedCount;
    j["directional_chamfer_pruned_count"] = result.directionalChamferPrunedCount;
    j["micro_adjust_total_ms"] = result.microAdjustTotalMs;
    j["micro_adjust_generate_ms"] = result.microAdjustGenerateMs;
    j["micro_adjust_score_ms"] = result.microAdjustScoreMs;
    j["micro_adjust_nms_ms"] = result.microAdjustNmsMs;
    j["micro_adjust_setup_ms"] = result.microAdjustSetupMs;
    j["micro_adjust_selected_count"] = result.microAdjustSelectedCount;
    j["micro_adjust_generated_count"] = result.microAdjustGeneratedCount;
    j["micro_adjust_scored_count"] = result.microAdjustScoredCount;
    j["micro_adjust_input_candidate_count"] = result.microAdjustInputCandidateCount;
    j["final_ranker_ms"] = result.finalRankerMs;
    j["report_write_ms"] = result.reportWriteMs;
    j["overlay_build_ms"] = result.overlayBuildMs;
    j["unaccounted_time_ms"] = result.unaccountedTimeMs;
    j["total_time_ms"] = result.totalTimeMs;
    j["recall"] = result.recall;
    j["precision"] = result.precision;
    j["f1"] = result.f1;
    std::ofstream out(config.reportDir / "latest_pipeline_v2_response_profile.json");
    if (out.is_open()) {
        out << j.dump(2);
    }

    std::ofstream csv(config.reportDir / "latest_pipeline_v2_profile.csv");
    if (csv.is_open()) {
        csv << "metric,value\n";
        csv << "totalTimeMs," << result.totalTimeMs << '\n';
        csv << "responseGenerationMs," << result.responseGenerationMs << '\n';
        csv << "responseVerificationMs," << result.responseVerificationMs << '\n';
        csv << "directionalChamferTimeMs," << result.directionalChamferTimeMs << '\n';
        csv << "candidateSelectionMs," << result.candidateSelectionMs << '\n';
        csv << "microAdjustTotalMs," << result.microAdjustTotalMs << '\n';
        csv << "microAdjustGenerateMs," << result.microAdjustGenerateMs << '\n';
        csv << "microAdjustScoreMs," << result.microAdjustScoreMs << '\n';
        csv << "microAdjustNmsMs," << result.microAdjustNmsMs << '\n';
        csv << "microAdjustSetupMs," << result.microAdjustSetupMs << '\n';
        csv << "microAdjustSelectedCount," << result.microAdjustSelectedCount << '\n';
        csv << "microAdjustGeneratedCount," << result.microAdjustGeneratedCount << '\n';
        csv << "roiDistanceFieldBuildMs," << result.roiDistanceFieldBuildMs << '\n';
        csv << "finalRankerMs," << result.finalRankerMs << '\n';
        csv << "reportWriteMs," << result.reportWriteMs << '\n';
        csv << "overlayBuildMs," << result.overlayBuildMs << '\n';
        csv << "unaccountedTimeMs," << result.unaccountedTimeMs << '\n';
    }
}

void writeMicroAdjustReports(const ShapeMatchPipelineV2Config& config, const ShapeMatchPipelineV2Result& result)
{
    std::filesystem::create_directories(config.reportDir);
    nlohmann::json j;
    j["enabled"] = config.responsePipeline.enableLevel0FinalPoseMicroAdjustment &&
                   config.responsePipeline.microAdjustMode != MicroAdjustMode::Disabled;
    j["mode"] = microAdjustModeName(config.responsePipeline.microAdjustMode);
    j["input_candidate_count"] = result.microAdjustInputCandidateCount;
    j["adjusted_candidate_count"] = result.microAdjustSelectedCount;
    j["generated_pose_count"] = result.microAdjustGeneratedCount;
    j["scored_pose_count"] = result.microAdjustScoredCount;
    j["kept_pose_count"] = result.microAdjustSelectedCount;
    j["max_children_per_candidate"] = config.responsePipeline.microAdjustMaxChildrenPerCandidate;
    j["max_total_micro_children"] = config.responsePipeline.microAdjustMaxTotalChildren;
    j["generate_ms"] = result.microAdjustGenerateMs;
    j["score_ms"] = result.microAdjustScoreMs;
    j["nms_ms"] = result.microAdjustNmsMs;
    j["total_ms"] = result.microAdjustTotalMs;
    std::ofstream jout(config.reportDir / "latest_micro_adjust_profile.json");
    if (jout.is_open()) {
        jout << j.dump(2);
    }

    std::ofstream txt(config.reportDir / "latest_micro_adjust_summary.txt");
    if (txt.is_open()) {
        txt << "Micro Adjust Summary\n";
        txt << "enabled: " << (j["enabled"].get<bool>() ? "true" : "false") << '\n';
        txt << "mode: " << microAdjustModeName(config.responsePipeline.microAdjustMode) << '\n';
        txt << "inputCandidateCount: " << result.microAdjustInputCandidateCount << '\n';
        txt << "adjustedCandidateCount: " << result.microAdjustSelectedCount << '\n';
        txt << "generatedPoseCount: " << result.microAdjustGeneratedCount << '\n';
        txt << "scoredPoseCount: " << result.microAdjustScoredCount << '\n';
        txt << "maxChildrenPerCandidate: " << config.responsePipeline.microAdjustMaxChildrenPerCandidate << '\n';
        txt << "maxTotalMicroChildren: " << config.responsePipeline.microAdjustMaxTotalChildren << '\n';
        txt << "generateMs: " << result.microAdjustGenerateMs << '\n';
        txt << "setupMs: " << result.microAdjustSetupMs << '\n';
        txt << "scoreMs: " << result.microAdjustScoreMs << '\n';
        txt << "nmsMs: " << result.microAdjustNmsMs << '\n';
        txt << "totalMs: " << result.microAdjustTotalMs << '\n';
    }

    std::ofstream csv(config.reportDir / "latest_micro_adjust_candidates.csv");
    if (csv.is_open()) {
        csv << "parent_rank,parent_x,parent_y,parent_theta_deg,child_x,child_y,child_theta_deg,score,kept,reject_reason\n";
        if (config.production.runtimeMode != ShapeMatchRuntimeMode::Debug) {
            csv << "disabled_in_production,0,0,0,0,0,0,0,false,production_summary_only\n";
        }
    }

    std::ofstream ab(config.reportDir / "latest_micro_adjust_ab.csv");
    if (ab.is_open()) {
        ab << "mode,total_ms,micro_adjust_total_ms,micro_generated,micro_scored,candidate_count,recall,precision,f1,tp,fp,fn,final_ranker_ms\n";
        const int tp = result.gtEvaluated && result.recall > 0.0
            ? static_cast<int>(std::round(result.recall * static_cast<double>(std::max(1, result.candidateCount))))
            : 0;
        const int fp = result.gtEvaluated && result.precision > 0.0
            ? std::max(0, result.candidateCount - static_cast<int>(std::round(result.precision * result.candidateCount)))
            : 0;
        ab << microAdjustModeName(config.responsePipeline.microAdjustMode) << ','
           << result.totalTimeMs << ','
           << result.microAdjustTotalMs << ','
           << result.microAdjustGeneratedCount << ','
           << result.microAdjustScoredCount << ','
           << result.candidateCount << ','
           << result.recall << ','
           << result.precision << ','
           << result.f1 << ','
           << tp << ','
           << fp << ','
           << 0 << ','
           << result.finalRankerMs << '\n';
    }
}

void writeR3VsR4Csv(const ShapeMatchPipelineV2Config& config, const ShapeMatchPipelineV2Result& result)
{
    std::filesystem::create_directories(config.reportDir);
    std::ofstream out(config.reportDir / "latest_pipeline_v2_directional_chamfer_vs_r3.csv");
    if (!out.is_open()) {
        return;
    }
    out << "case_name,method,total_time_ms,response_time_ms,fast_verify_time_ms,directional_chamfer_time_ms,final_ranker_time_ms,candidate_before_chamfer,candidate_after_chamfer,final_ranker_input,recall,precision,f1,top1_hit,top3_hit,top5_hit,tp,fp,fn\n";
    out << "pipeline_v2,R4DirectionalChamfer,"
        << result.totalTimeMs << ','
        << result.responseGenerationMs << ','
        << result.responseVerificationMs << ','
        << result.directionalChamferTimeMs << ','
        << result.finalRankerMs << ','
        << result.directionalChamferInputCount << ','
        << result.directionalChamferAcceptedCount << ','
        << result.selectedCandidateCount << ','
        << result.recall << ','
        << result.precision << ','
        << result.f1 << ','
        << result.top1Hit << ','
        << result.top3Hit << ','
        << result.top5Hit << ','
        << 0 << ',' << 0 << ',' << 0 << '\n';
}

void writeVerifiedCandidatesCsv(const ShapeMatchPipelineV2Config& config,
                                const std::vector<VerifiedResponsePeak>& candidates,
                                double levelScale)
{
    std::filesystem::create_directories(config.reportDir);
    std::ofstream out(config.reportDir / "latest_pipeline_v2_response_candidates.csv");
    if (!out.is_open()) {
        return;
    }
    out << "rank,x,y,theta_deg,scale,response_score,fast_score,combined_score,source_level,tile_row,tile_col\n";
    for (size_t i = 0; i < candidates.size(); ++i) {
        const VerifiedResponsePeak& c = candidates[i];
        out << (i + 1) << ','
            << c.pose.x / levelScale << ','
            << c.pose.y / levelScale << ','
            << c.pose.thetaDeg() << ','
            << c.pose.scale << ','
            << c.responseScore << ','
            << c.fastScore << ','
            << c.combinedScore << ','
            << config.responsePipeline.responsePyramidLevel << ','
            << c.peak.tileRow << ','
            << c.peak.tileCol << '\n';
    }
}

void writeRegionsCsv(const ShapeMatchPipelineV2Config& config, const std::vector<ResponseCandidateRegion>& regions)
{
    std::filesystem::create_directories(config.reportDir);
    std::ofstream out(config.reportDir / "latest_pipeline_v2_response_regions.csv");
    if (!out.is_open()) {
        return;
    }
    out << "region_id,roi_x,roi_y,roi_w,roi_h,seed_x,seed_y,seed_theta_deg,seed_score,source_peak_count\n";
    for (const ResponseCandidateRegion& r : regions) {
        out << r.regionId << ','
            << r.roiLevel0.x << ','
            << r.roiLevel0.y << ','
            << r.roiLevel0.width << ','
            << r.roiLevel0.height << ','
            << r.seedPoseLevel0.x << ','
            << r.seedPoseLevel0.y << ','
            << r.seedPoseLevel0.thetaDeg() << ','
            << r.seedScore << ','
            << r.sourcePeakCount << '\n';
    }
}

void writeVsOldCsv(const ShapeMatchPipelineV2Config& config, const ShapeMatchPipelineV2Result& result)
{
    std::filesystem::create_directories(config.reportDir);
    std::ofstream out(config.reportDir / "latest_pipeline_v2_vs_old_pipeline.csv");
    if (!out.is_open()) {
        return;
    }
    out << "case_name,method,total_time_ms,response_generation_ms,local_refine_ms,final_ranker_ms,candidate_count,level0_raw_children,level0_scored_children,recall,precision,f1,top1_hit,top3_hit,top5_hit,top10_hit,tp,fp,fn,avg_dxy,avg_dtheta\n";
    out << "pipeline_v2_response,PipelineV2OrientationResponse,"
        << result.totalTimeMs << ','
        << result.responseGenerationMs << ','
        << 0.0 << ','
        << result.finalRankerMs << ','
        << result.selectedCandidateCount << ','
        << result.oldLevel0RawChildren << ','
        << result.oldLevel0ScoredChildren << ','
        << result.recall << ','
        << result.precision << ','
        << result.f1 << ','
        << result.top1Hit << ','
        << result.top3Hit << ','
        << result.top5Hit << ','
        << result.top10Hit << ','
        << 0 << ',' << 0 << ',' << 0 << ',' << 0.0 << ',' << 0.0 << '\n';
}

std::vector<MatchPose> toLevel0Poses(const std::vector<VerifiedResponsePeak>& selected, double scale)
{
    std::vector<MatchPose> poses;
    poses.reserve(selected.size());
    const double safeScale = std::max(1e-9, scale);
    for (const VerifiedResponsePeak& peak : selected) {
        MatchPose pose = peak.pose;
        pose.x /= safeScale;
        pose.y /= safeScale;
        poses.push_back(pose);
    }
    return poses;
}

std::vector<MatchPose> microAdjustLevel0FinalPoses(const std::vector<MatchPose>& poses,
                                                   const ShapeMatchPipelineV2Context& context,
                                                   const ShapeMatchPipelineV2Config& config,
                                                   ShapeMatchPipelineV2Result& result)
{
    const auto totalStart = std::chrono::steady_clock::now();
    if (!config.responsePipeline.enableLevel0FinalPoseMicroAdjustment ||
        config.responsePipeline.microAdjustMode == MicroAdjustMode::Disabled ||
        poses.empty() ||
        context.templateModel == nullptr ||
        context.edgeData == nullptr) {
        return poses;
    }

    result.microAdjustInputCandidateCount = std::min<int>(
        static_cast<int>(poses.size()),
        std::max(1, config.responsePipeline.microAdjustMaxInputCandidates));
    const int radius = std::max(0, config.responsePipeline.finalPoseMicroAdjustRadiusPx);
    const int step = std::max(1, config.responsePipeline.finalPoseMicroAdjustStepPx);
    const double angleRadius = std::max(0.0, config.responsePipeline.finalPoseMicroAdjustAngleRadiusDeg);
    const double angleStep = std::max(0.1, config.responsePipeline.finalPoseMicroAdjustAngleStepDeg);
    const int maxChildrenPerCandidate = std::max(1, config.responsePipeline.microAdjustMaxChildrenPerCandidate);
    const int maxTotalChildren = std::max(1, config.responsePipeline.microAdjustMaxTotalChildren);

    const auto generateStart = std::chrono::steady_clock::now();
    std::vector<int> offsets;
    offsets.push_back(0);
    if (radius > 0 && maxChildrenPerCandidate > 1) {
        offsets.push_back(-radius);
        offsets.push_back(radius);
    }
    std::vector<double> angleOffsets;
    angleOffsets.push_back(0.0);
    if (angleRadius > 1e-6 && maxChildrenPerCandidate > 1) {
        angleOffsets.push_back(-std::min(angleRadius, angleStep));
        angleOffsets.push_back(std::min(angleRadius, angleStep));
    }
    result.microAdjustGenerateMs = elapsedMs(generateStart);

    const auto setupStart = std::chrono::steady_clock::now();
    std::vector<const TemplatePoint*> microPoints;
    if (config.responsePipeline.microAdjustUseDirectionalChamfer) {
        const int maxPoints = std::max(16, config.responsePipeline.microAdjustMaxTemplatePoints);
        const int stride = std::max(1, static_cast<int>(context.templateModel->points.size()) / maxPoints);
        microPoints.reserve(static_cast<size_t>(maxPoints));
        for (size_t i = 0; i < context.templateModel->points.size() && static_cast<int>(microPoints.size()) < maxPoints; i += static_cast<size_t>(stride)) {
            microPoints.push_back(&context.templateModel->points[i]);
        }
    }
    result.microAdjustSetupMs = elapsedMs(setupStart);

    FastPoseScorer scorer;
    FastScoreContext scoreContext;
    auto fastDirectionalScore = [&](const MatchPose& trial) {
        if (!config.responsePipeline.microAdjustUseDirectionalChamfer ||
            microPoints.empty() ||
            context.edgeData->distanceMap.empty()) {
            return scorer.scorePose(*context.templateModel, *context.edgeData, trial, 0, scoreContext).fastScore;
        }
        const double c = std::cos(trial.theta);
        const double s = std::sin(trial.theta);
        double scoreSum = 0.0;
        double weightSum = 0.0;
        const double distSigma = std::max(0.1, config.responsePipeline.microAdjustDistanceSigmaPx);
        const double maxDist = std::max(0.1, config.responsePipeline.microAdjustMaxDistancePx);
        const double oriSigma = std::max(0.1, config.responsePipeline.microAdjustOrientationSigmaDeg);
        for (const TemplatePoint* pt : microPoints) {
            const double x = trial.x + trial.scale * (c * pt->position.x - s * pt->position.y);
            const double y = trial.y + trial.scale * (s * pt->position.x + c * pt->position.y);
            const int ix = static_cast<int>(std::round(x));
            const int iy = static_cast<int>(std::round(y));
            const double w = std::max(0.001, pt->weight);
            weightSum += w;
            if (ix < 0 || iy < 0 || ix >= context.edgeData->distanceMap.cols || iy >= context.edgeData->distanceMap.rows) {
                continue;
            }
            const double d = std::min<double>(context.edgeData->distanceMap.at<float>(iy, ix), maxDist);
            const double ds = std::exp(-(d * d) / (2.0 * distSigma * distSigma));
            double os = 1.0;
            if (!context.edgeData->orientationMap.empty()) {
                const double expected = std::atan2(s * pt->normal.x + c * pt->normal.y,
                                                   c * pt->normal.x - s * pt->normal.y);
                const double actual = context.edgeData->orientationMap.at<float>(iy, ix);
                const double ad = std::abs(radToDeg(wrapToPi(expected - actual)));
                os = std::exp(-(ad * ad) / (2.0 * oriSigma * oriSigma));
            }
            scoreSum += w * (0.65 * ds + 0.35 * os);
        }
        return weightSum > 0.0 ? scoreSum / weightSum : 0.0;
    };
    std::vector<MatchPose> adjusted;
    adjusted.reserve(poses.size());
    const auto scoreStart = std::chrono::steady_clock::now();
    int globalChildren = 0;
    for (size_t poseIndex = 0; poseIndex < poses.size(); ++poseIndex) {
        const MatchPose& pose = poses[poseIndex];
        if (static_cast<int>(poseIndex) >= result.microAdjustInputCandidateCount) {
            adjusted.push_back(pose);
            continue;
        }
        MatchPose bestPose = pose;
        double bestScore = -1.0;
        int childCount = 0;
        auto scoreTrial = [&](double dx, double dy, double da) {
            if (childCount >= maxChildrenPerCandidate || globalChildren >= maxTotalChildren) {
                return;
            }
            ++childCount;
            ++globalChildren;
            ++result.microAdjustGeneratedCount;
            ++result.microAdjustScoredCount;
            MatchPose trial = pose;
            trial.x += dx;
            trial.y += dy;
            trial.theta = wrapToPi(trial.theta + degToRad(da));
            const double score = fastDirectionalScore(trial);
            if (score > bestScore) {
                bestScore = score;
                bestPose = trial;
            }
        };
        for (double da : angleOffsets) {
            scoreTrial(0.0, 0.0, da);
        }
        if (childCount < maxChildrenPerCandidate && radius > 0) {
            for (double da : angleOffsets) {
                scoreTrial(static_cast<double>(-radius), 0.0, da);
                scoreTrial(static_cast<double>(radius), 0.0, da);
                scoreTrial(0.0, static_cast<double>(-radius), da);
                scoreTrial(0.0, static_cast<double>(radius), da);
                scoreTrial(static_cast<double>(-radius), static_cast<double>(-radius), da);
                scoreTrial(static_cast<double>(radius), static_cast<double>(-radius), da);
                scoreTrial(static_cast<double>(-radius), static_cast<double>(radius), da);
                scoreTrial(static_cast<double>(radius), static_cast<double>(radius), da);
                if (childCount >= maxChildrenPerCandidate || globalChildren >= maxTotalChildren) {
                    break;
                }
            }
        }
        adjusted.push_back(bestPose);
    }
    result.microAdjustScoreMs = elapsedMs(scoreStart);
    result.microAdjustSelectedCount = result.microAdjustInputCandidateCount;
    result.microAdjustTotalMs = elapsedMs(totalStart);
    return adjusted;
}

std::vector<ScoredCandidate> filterFinalOutputCandidates(const std::vector<ScoredCandidate>& ranked,
                                                         const ShapeMatchPipelineV2Config& config)
{
    if (ranked.empty()) {
        return {};
    }

    const double bestScore = ranked.front().score.finalScore;
    const double minScore = std::max(config.responsePipeline.minOutputFinalScore,
                                     bestScore * config.responsePipeline.minOutputScoreRelativeToBest);
    const int maxCount = std::max(1, config.responsePipeline.maxOutputCandidates);
    std::vector<ScoredCandidate> filtered;
    filtered.reserve(std::min<int>(maxCount, static_cast<int>(ranked.size())));
    auto suppressedByOutputNms = [&](const ScoredCandidate& candidate) {
        for (const ScoredCandidate& kept : filtered) {
            const double dx = candidate.pose.x - kept.pose.x;
            const double dy = candidate.pose.y - kept.pose.y;
            const double dxy = std::sqrt(dx * dx + dy * dy);
            if (dxy >= config.responsePipeline.finalOutputNmsPx) {
                continue;
            }
            if (config.responsePipeline.finalOutputNmsIgnoreAngle) {
                return true;
            }
            if (poseDthetaDeg(candidate.pose, kept.pose) <= config.responsePipeline.responseCandidateNmsThetaDeg) {
                return true;
            }
        }
        return false;
    };

    for (const ScoredCandidate& candidate : ranked) {
        if (static_cast<int>(filtered.size()) >= maxCount) {
            break;
        }
        if (config.responsePipeline.outputOnlyAcceptedFinalCandidates && !candidate.score.accepted) {
            continue;
        }
        if (candidate.score.finalScore < minScore) {
            continue;
        }
        if (suppressedByOutputNms(candidate)) {
            continue;
        }
        filtered.push_back(candidate);
    }

    if (filtered.empty()) {
        filtered.push_back(ranked.front());
    }
    for (size_t i = 0; i < filtered.size(); ++i) {
        filtered[i].rank = static_cast<int>(i) + 1;
    }
    return filtered;
}

void applyMultiTargetResult(ShapeMatchPipelineV2Result& result, const MultiTargetEvalResult& eval)
{
    result.gtEvaluated = true;
    result.recall = eval.recall;
    result.precision = eval.precision;
    result.f1 = eval.f1;
    result.top1Hit = eval.top1Hit;
    result.top3Hit = eval.top3Hit;
    result.top5Hit = eval.top5Hit;
    result.top10Hit = eval.top10Hit;
}

double accountedPipelineMs(const ShapeMatchPipelineV2Result& result)
{
    return result.responseGenerationMs
        + result.responseVerificationMs
        + result.directionalChamferFieldBuildTimeMs
        + result.directionalChamferTimeMs
        + result.candidateSelectionMs
        + result.regionGenerationMs
        + result.roiDistanceFieldBuildMs
        + result.microAdjustTotalMs
        + result.finalRankerMs
        + result.reportWriteMs
        + result.overlayBuildMs;
}

void writeCandidatesCsv(const ShapeMatchPipelineV2Config& config, const ShapeMatchPipelineV2Result& result)
{
    std::filesystem::create_directories(config.reportDir);
    const std::filesystem::path path = config.reportDir / "latest_pipeline_v2_candidates.csv";
    std::ofstream out(path);
    if (!out.is_open()) {
        return;
    }

    out << "rank,x,y,theta_deg,scale\n";
    for (size_t i = 0; i < result.candidates.size(); ++i) {
        const MatchPose& pose = result.candidates[i];
        out << (i + 1) << ','
            << pose.x << ','
            << pose.y << ','
            << pose.thetaDeg() << ','
            << pose.scale << '\n';
    }
}

const ShapeTemplateModel* selectTemplateModel(const ShapeMatchPipelineV2Context& context, int level)
{
    if (context.templatePyramid != nullptr && level >= 0 && level < context.templatePyramid->levelCount()) {
        return &context.templatePyramid->level(level);
    }
    return level == 0 ? context.templateModel : nullptr;
}

const EdgeImageData* selectEdgeData(const ShapeMatchPipelineV2Context& context, int level)
{
    if (context.imagePyramid != nullptr && level >= 0 && level < context.imagePyramid->levelCount()) {
        return &context.imagePyramid->level(level);
    }
    return level == 0 ? context.edgeData : nullptr;
}

bool validateContext(const ShapeMatchPipelineV2Context& context,
                     const ShapeMatchPipelineV2Config& config,
                     ShapeMatchPipelineV2Result& result)
{
    if (context.templateModel == nullptr) {
        result.failureReason = "Missing templateModel";
        return false;
    }
    if (context.edgeData == nullptr) {
        result.failureReason = "Missing edgeData";
        return false;
    }
    if (context.templateModel->empty()) {
        result.failureReason = "Empty templateModel";
        return false;
    }
    if (context.edgeData->edgeMap.empty() && context.edgeData->edgePoints.empty()) {
        result.failureReason = "Empty edgeData";
        return false;
    }

    if (config.pyramidLevel > 0) {
        if (!hasLevel(context.imagePyramid, config.pyramidLevel)) {
            result.failureReason = "Missing imagePyramid level";
            return false;
        }
        if (!hasLevel(context.templatePyramid, config.pyramidLevel)) {
            result.failureReason = "Missing templatePyramid level";
            return false;
        }
    }
    return true;
}

void microAdjustVerifiedPeaks(std::vector<VerifiedResponsePeak>& verified,
                              const ShapeMatchPipelineV2Context& context,
                              const ShapeMatchPipelineV2Config& config,
                              int level)
{
    if (!config.responsePipeline.enableMicroAdjustment || verified.empty()) {
        return;
    }
    const ShapeTemplateModel* model = selectTemplateModel(context, level);
    const EdgeImageData* edgeData = selectEdgeData(context, level);
    if (model == nullptr || edgeData == nullptr) {
        return;
    }

    const int radius = std::max(0, config.responsePipeline.microAdjustRadiusPx);
    const double angleRadius = std::max(0.0, config.responsePipeline.microAdjustAngleRadiusDeg);
    const int maxChildren = std::max(1, config.responsePipeline.maxMicroAdjustChildrenPerPeak);
    std::vector<cv::Point2d> offsets;
    offsets.push_back({0.0, 0.0});
    if (radius > 0 && maxChildren > 1) {
        offsets.push_back({static_cast<double>(-radius), 0.0});
        offsets.push_back({static_cast<double>(radius), 0.0});
        offsets.push_back({0.0, static_cast<double>(-radius)});
        offsets.push_back({0.0, static_cast<double>(radius)});
    }
    if (radius > 1 && maxChildren > 15) {
        offsets.push_back({static_cast<double>(-radius), static_cast<double>(-radius)});
        offsets.push_back({static_cast<double>(radius), static_cast<double>(-radius)});
        offsets.push_back({static_cast<double>(-radius), static_cast<double>(radius)});
        offsets.push_back({static_cast<double>(radius), static_cast<double>(radius)});
    }

    std::vector<double> angleOffsets;
    angleOffsets.push_back(0.0);
    if (angleRadius > 1e-6 && maxChildren > 1) {
        angleOffsets.push_back(-angleRadius);
        angleOffsets.push_back(angleRadius);
        if (maxChildren > static_cast<int>(offsets.size()) * 3 + 2) {
            angleOffsets.push_back(-0.5 * angleRadius);
            angleOffsets.push_back(0.5 * angleRadius);
        }
    }

    FastPoseScorer scorer;
    FastScoreContext scoreContext;
    for (VerifiedResponsePeak& peak : verified) {
        VerifiedResponsePeak best = peak;
        double bestScore = peak.fastScore;
        int evaluated = 0;
        for (const cv::Point2d& offset : offsets) {
            for (double angleOffsetDeg : angleOffsets) {
                if (evaluated >= maxChildren) {
                    break;
                }
                ++evaluated;
                MatchPose pose = peak.pose;
                pose.x += offset.x;
                pose.y += offset.y;
                pose.theta = wrapToPi(pose.theta + degToRad(angleOffsetDeg));
                const CoarseCandidate scored = scorer.scorePose(*model, *edgeData, pose, level, scoreContext);
                if (scored.fastScore > bestScore) {
                    best.pose = pose;
                    best.fastScore = scored.fastScore;
                    best.coverageApprox = scored.coverageApprox;
                    best.orientationApprox = scored.orientationApprox;
                    best.polarityApprox = scored.polarityApprox;
                    best.accepted = scored.fastScore >= config.responsePipeline.minVerifiedFastScore;
                    best.rejectReason = best.accepted ? "" : "fast_score_below_threshold";
                    bestScore = scored.fastScore;
                }
            }
            if (evaluated >= maxChildren) {
                break;
            }
        }
        peak = best;
    }
}

void runOrientationResponsePipeline(const ShapeMatchPipelineV2Context& context,
                                    const ShapeMatchPipelineV2Config& config,
                                    ShapeMatchPipelineV2Result& result)
{
    static TemplateRuntimeCache templateCache;
    static DirectionalFieldCache fieldCache;
    const int level = config.responsePipeline.responsePyramidLevel;
    const ShapeTemplateModel* model = selectTemplateModel(context, level);
    const EdgeImageData* edgeData = selectEdgeData(context, level);
    if (model == nullptr || edgeData == nullptr) {
        result.failureReason = "Invalid response pyramid level or missing pyramid data";
        return;
    }

    OrientationResponseConfig responseConfig = config.orientationResponseConfig;
    if (config.production.runtimeMode != ShapeMatchRuntimeMode::Debug) {
        responseConfig.exportResponseDebugImages = false;
        responseConfig.exportResponseCsv = false;
        responseConfig.exportResponseJson = false;
        responseConfig.exportResponseSummary = false;
    }
    responseConfig.maxTotalPeaks = config.responsePipeline.maxResponsePeaks;
    ResponseCandidateGenerator responseGenerator(responseConfig);
    ResponseDebugReport responseReport;
    const auto responseStart = std::chrono::steady_clock::now();
    std::vector<ResponsePeak> peaks = responseGenerator.generatePeaks(*model, *edgeData, level, &responseReport);
    result.responseGenerationMs = elapsedMs(responseStart);
    result.candidateGenerationMs = result.responseGenerationMs;
    result.responsePeakCount = static_cast<int>(peaks.size());

    if (peaks.empty()) {
        result.failureReason = "Orientation response generated no peaks";
        return;
    }

    if (config.responsePipeline.executionMode == PipelineV2ExecutionMode::CandidateOnly) {
        result.candidates.reserve(peaks.size());
        for (const ResponsePeak& peak : peaks) {
            result.candidates.push_back(peak.pose);
        }
        result.selectedCandidateCount = static_cast<int>(result.candidates.size());
        return;
    }

    std::vector<VerifiedResponsePeak> verified;
    if (config.responsePipeline.enableResponsePeakVerification) {
        ResponsePeakVerifier verifier;
        verified = verifier.verify(peaks, context, config, &result.responseVerificationMs);
    } else {
        verified.reserve(peaks.size());
        for (const ResponsePeak& peak : peaks) {
            VerifiedResponsePeak item;
            item.peak = peak;
            item.pose = peak.pose;
            item.responseScore = peak.responseScore;
            item.fastScore = peak.responseScore;
            item.accepted = true;
            verified.push_back(item);
        }
    }
    result.verifiedResponsePeakCount = static_cast<int>(verified.size());

    if (config.directionalChamfer.enableDirectionalChamferVerification && !verified.empty()) {
        const SegmentTemplate* segmentTemplatePtr = nullptr;
        SegmentTemplate localSegmentTemplate;
        if (config.production.enableRuntimeCaches) {
            segmentTemplatePtr = &templateCache.getOrBuildSegmentTemplate(*model, level, config.directionalChamfer);
            result.segmentTemplateCacheHits = templateCache.segmentTemplateHitCount();
            result.segmentTemplateCacheMisses = templateCache.segmentTemplateMissCount();
        } else {
            SegmentTemplateBuilder segmentBuilder;
            localSegmentTemplate = segmentBuilder.build(*model, level, config.directionalChamfer);
            segmentTemplatePtr = &localSegmentTemplate;
        }
        const DirectionalDistanceField* fieldPtr = nullptr;
        DirectionalDistanceField localField;
        if (config.production.enableRuntimeCaches) {
            fieldPtr = &fieldCache.getOrBuildDirectionalField(*edgeData, level, config.directionalChamfer);
            result.directionalFieldCacheHits = fieldCache.directionalFieldHitCount();
            result.directionalFieldCacheMisses = fieldCache.directionalFieldMissCount();
        } else {
            DirectionalDistanceFieldBuilder fieldBuilder;
            localField = fieldBuilder.build(*edgeData, level, config.directionalChamfer);
            fieldPtr = &localField;
        }
        const SegmentTemplate& segmentTemplate = *segmentTemplatePtr;
        const DirectionalDistanceField& field = *fieldPtr;
        result.directionalChamferFieldBuildTimeMs = field.buildTimeMs;
        result.directionalChamferInputCount = static_cast<int>(verified.size());
        if (!segmentTemplate.empty() && field.valid) {
            DirectionalChamferVerifier verifier(config.directionalChamfer);
            std::vector<DirectionalChamferScore> chamferScores;
            int prunedCount = 0;
            const auto chamferStart = std::chrono::steady_clock::now();
            std::vector<VerifiedResponsePeak> chamferVerified;
            const bool heavyDebug = config.production.runtimeMode == ShapeMatchRuntimeMode::Debug &&
                                    config.directionalChamfer.exportDirectionalChamferReport;
            if (config.production.enableParallelChamferVerification && !heavyDebug) {
                ParallelDirectionalChamferVerifier parallelVerifier(config.production);
                chamferVerified = parallelVerifier.verify(verified,
                                                          segmentTemplate,
                                                          field,
                                                          verifier,
                                                          config.directionalChamfer.targetCandidatesAfterVerify,
                                                          &prunedCount);
            } else {
                chamferVerified = verifier.verifyPeaks(verified, segmentTemplate, field, &chamferScores, &prunedCount);
            }
            result.directionalChamferTimeMs = elapsedMs(chamferStart);
            result.directionalChamferAcceptedCount = static_cast<int>(chamferVerified.size());
            result.directionalChamferPrunedCount = prunedCount;

            if (config.production.runtimeMode == ShapeMatchRuntimeMode::Debug &&
                config.directionalChamfer.exportDirectionalChamferReport) {
                DirectionalChamferDebugReport chamferReport;
                chamferReport.inputCandidates = result.directionalChamferInputCount;
                chamferReport.acceptedCandidates = result.directionalChamferAcceptedCount;
                chamferReport.prunedCandidates = result.directionalChamferPrunedCount;
                chamferReport.finalCandidatesToRanker = result.directionalChamferAcceptedCount;
                chamferReport.fieldBuildTimeMs = result.directionalChamferFieldBuildTimeMs;
                chamferReport.verifyTimeMs = result.directionalChamferTimeMs;
                chamferReport.avgTimePerCandidateMs =
                    chamferReport.inputCandidates > 0 ? chamferReport.verifyTimeMs / chamferReport.inputCandidates : 0.0;
                chamferReport.usedOldLocalRefine = false;
                const int debugCount = std::min<int>(static_cast<int>(verified.size()), static_cast<int>(chamferScores.size()));
                chamferReport.candidates.reserve(static_cast<size_t>(debugCount));
                for (int i = 0; i < debugCount; ++i) {
                    chamferReport.topScore = std::max(chamferReport.topScore, chamferScores[static_cast<size_t>(i)].finalScore);
                    chamferReport.candidates.push_back({verified[static_cast<size_t>(i)], chamferScores[static_cast<size_t>(i)]});
                }
                DirectionalChamferReportWriter(config.reportDir).write(chamferReport);
            }

            if (!chamferVerified.empty()) {
                verified = std::move(chamferVerified);
                result.verifiedResponsePeakCount = static_cast<int>(verified.size());
            }
        } else {
            result.directionalChamferAcceptedCount = static_cast<int>(verified.size());
        }
    }

    microAdjustVerifiedPeaks(verified, context, config, level);

    if (config.responsePipeline.executionMode == PipelineV2ExecutionMode::VerifyCandidates) {
        for (const VerifiedResponsePeak& peak : verified) {
            result.candidates.push_back(peak.pose);
        }
        result.selectedCandidateCount = static_cast<int>(result.candidates.size());
        return;
    }

    ResponsePeakCandidateSelector selector;
    std::vector<VerifiedResponsePeak> selected = selector.select(verified, context, config, &result.candidateSelectionMs);
    result.selectedCandidateCount = static_cast<int>(selected.size());
    if (selected.empty()) {
        result.failureReason = "No verified response candidates survived selection";
        return;
    }

    ResponsePeakRegionGenerator regionGenerator;
    std::vector<ResponseCandidateRegion> regions = regionGenerator.generateRegions(selected, context, config, &result.regionGenerationMs);
    result.candidateRegionCount = static_cast<int>(regions.size());

    const double scale = levelScale(context, level);
    result.candidates = toLevel0Poses(selected, scale);
    result.candidates = microAdjustLevel0FinalPoses(result.candidates, context, config, result);

    if (config.responsePipeline.enableFinalRanker && context.templateModel != nullptr && context.edgeData != nullptr) {
        ShapeMatchEvalConfig evalConfig;
        evalConfig.topK = std::max(1, config.responsePipeline.targetFinalCandidates);
        evalConfig.maxFinalRankerInputCandidates = std::max(1, config.responsePipeline.maxFinalCandidates);
        evalConfig.targetFinalRankerInputCandidates = std::max(1, config.responsePipeline.targetFinalCandidates);
        evalConfig.xyOkThresholdPx = 5.0;
        evalConfig.angleOkThresholdDeg = 3.5;
        evalConfig.minCoverageRatio = 0.32;
        evalConfig.minInlierRatio = 0.22;
        evalConfig.maxMedianErrorPx = 3.5;
        evalConfig.maxP90ErrorPx = 7.0;
        evalConfig.enableAnglePeriodEquivalence = config.symmetry.enableSymmetrySuppression &&
                                                  config.symmetry.assume180DegreeSymmetry;
        evalConfig.angleEquivalencePeriodDeg = config.symmetry.symmetryAngleDeg;
        if (config.production.runtimeMode != ShapeMatchRuntimeMode::Debug &&
            config.production.productionDisablePointEval) {
            evalConfig.enablePointEvaluations = false;
            evalConfig.exportPointEvalCsv = false;
        }
        evalConfig.enableParallelFinalRanking = config.production.enableParallelFinalRanking;
        evalConfig.finalRankerNumThreads = config.production.numWorkerThreads;
        CandidateRanker ranker(evalConfig);
        FinalRankerProfile profile;
        const auto rankStart = std::chrono::steady_clock::now();
        std::vector<ScoredCandidate> ranked = ranker.rank(result.candidates,
                                                          *context.templateModel,
                                                          *context.edgeData,
                                                          config.enableGtEvaluation && !context.groundTruth.empty() ? &context.groundTruth : nullptr,
                                                          &profile);
        result.finalRankerMs = elapsedMs(rankStart);
        ranked = filterFinalOutputCandidates(ranked, config);
        result.candidates.clear();
        result.candidates.reserve(ranked.size());
        for (const ScoredCandidate& candidate : ranked) {
            result.candidates.push_back(candidate.pose);
        }
        if (config.responsePipeline.enableGtEvaluation && !context.groundTruth.empty()) {
            MultiTargetEvaluator evaluator(evalConfig);
            applyMultiTargetResult(result, evaluator.evaluate(ranked, context.groundTruth));
        }
    }

    writeVerifiedCandidatesCsv(config, selected, scale);
    writeRegionsCsv(config, regions);
}

} // namespace

const char* candidateGenerationModeName(CandidateGenerationMode mode)
{
    switch (mode) {
    case CandidateGenerationMode::OldVoting:
        return "OldVoting";
    case CandidateGenerationMode::OrientationResponse:
        return "OrientationResponse";
    case CandidateGenerationMode::CompareOldAndResponse:
        return "CompareOldAndResponse";
    }
    return "Unknown";
}

const char* pipelineV2ExecutionModeName(PipelineV2ExecutionMode mode)
{
    switch (mode) {
    case PipelineV2ExecutionMode::CandidateOnly:
        return "CandidateOnly";
    case PipelineV2ExecutionMode::VerifyCandidates:
        return "VerifyCandidates";
    case PipelineV2ExecutionMode::FullCoarseMatch:
        return "FullCoarseMatch";
    case PipelineV2ExecutionMode::CompareWithOldPipeline:
        return "CompareWithOldPipeline";
    }
    return "Unknown";
}

const char* shapeMatchRuntimeModeName(ShapeMatchRuntimeMode mode)
{
    switch (mode) {
    case ShapeMatchRuntimeMode::Debug:
        return "Debug";
    case ShapeMatchRuntimeMode::Production:
        return "Production";
    case ShapeMatchRuntimeMode::Benchmark:
        return "Benchmark";
    }
    return "Unknown";
}

ShapeMatchPipelineV2::ShapeMatchPipelineV2(ShapeMatchPipelineV2Config config)
    : config_(std::move(config))
{
}

ShapeMatchPipelineV2Result ShapeMatchPipelineV2::run(const ShapeMatchPipelineV2Context& context)
{
    const auto start = std::chrono::steady_clock::now();
    ShapeMatchPipelineV2Result result;
    result.mode = config_.candidateGenerationMode;

    if (!validateContext(context, config_, result)) {
        result.totalTimeMs = elapsedMs(start);
        if (config_.enableDebugReport) {
            writeTextSummary(config_, context, result);
            writeCandidatesCsv(config_, result);
        }
        return result;
    }

    if (config_.candidateGenerationMode == CandidateGenerationMode::OldVoting) {
        std::unique_ptr<IShapeCandidateGenerator> generator;
        generator = std::make_unique<OldVotingCandidateGeneratorAdapter>();
        result.candidates = generator->generate(context, config_, &result);
    } else if (config_.candidateGenerationMode == CandidateGenerationMode::OrientationResponse) {
        runOrientationResponsePipeline(context, config_, result);
    } else {
        std::unique_ptr<IShapeCandidateGenerator> generator;
        generator = std::make_unique<OldVotingCandidateGeneratorAdapter>();
        result.failureReason = "CompareOldAndResponse warning: returned OldVoting candidates; response pipeline report generated separately";
        result.candidates = generator->generate(context, config_, &result);
        ShapeMatchPipelineV2Result responseOnly;
        responseOnly.mode = CandidateGenerationMode::OrientationResponse;
        runOrientationResponsePipeline(context, config_, responseOnly);
    }
    result.candidateCount = static_cast<int>(result.candidates.size());
    result.ok = (result.failureReason.empty() && result.candidateCount > 0) ||
                config_.candidateGenerationMode == CandidateGenerationMode::CompareOldAndResponse;

    if (config_.enableGtEvaluation && !context.groundTruth.empty()) {
        const double scale = levelScale(context, config_.pyramidLevel);
        result.gtEvaluated = true;
        result.recall = recallAtAllCandidates(result.candidates, context.groundTruth, scale);
        result.top1Hit = topKHit(result.candidates, context.groundTruth, 1, scale);
        result.top3Hit = topKHit(result.candidates, context.groundTruth, 3, scale);
        result.top5Hit = topKHit(result.candidates, context.groundTruth, 5, scale);
        result.top10Hit = topKHit(result.candidates, context.groundTruth, 10, scale);
    }

    result.totalTimeMs = elapsedMs(start);
    result.unaccountedTimeMs = std::max(0.0, result.totalTimeMs - accountedPipelineMs(result));

    if (config_.enableDebugReport) {
        const auto reportStart = std::chrono::steady_clock::now();
        writeTextSummary(config_, context, result);
        writeCandidatesCsv(config_, result);
        if (config_.candidateGenerationMode == CandidateGenerationMode::OrientationResponse ||
            config_.candidateGenerationMode == CandidateGenerationMode::CompareOldAndResponse) {
            if (config_.production.runtimeMode == ShapeMatchRuntimeMode::Production) {
                writeProductionSummary(config_, context, result);
                writeResponseProfileJson(config_, result);
            } else if (config_.production.runtimeMode == ShapeMatchRuntimeMode::Debug) {
                writeResponseSummary(config_, context, result);
                writeResponseProfileJson(config_, result);
                writeVsOldCsv(config_, result);
                writeR3VsR4Csv(config_, result);
            }
            writeMicroAdjustReports(config_, result);
        }
        result.reportWriteMs = elapsedMs(reportStart);
        result.totalTimeMs = elapsedMs(start);
        result.unaccountedTimeMs = std::max(0.0, result.totalTimeMs - accountedPipelineMs(result));
        writeTextSummary(config_, context, result);
        if (config_.candidateGenerationMode == CandidateGenerationMode::OrientationResponse ||
            config_.candidateGenerationMode == CandidateGenerationMode::CompareOldAndResponse) {
            if (config_.production.runtimeMode == ShapeMatchRuntimeMode::Production) {
                writeProductionSummary(config_, context, result);
            } else if (config_.production.runtimeMode == ShapeMatchRuntimeMode::Debug) {
                writeResponseSummary(config_, context, result);
            }
            writeResponseProfileJson(config_, result);
            writeMicroAdjustReports(config_, result);
        }
    }

    return result;
}

} // namespace ShapeMatch
