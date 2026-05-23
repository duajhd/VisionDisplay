#include "shape_match/tests/ShapeMatchEvalSelfTest.h"

#include "shape_match/core/EdgeImageData.h"
#include "shape_match/evaluation/CandidateRanker.h"
#include "shape_match/evaluation/MatchEvaluator.h"
#include "shape_match/evaluation/MultiTargetEvaluator.h"
#include "shape_match/evaluation/PoseErrorEvaluator.h"
#include "shape_match/io/MatchReportWriter.h"
#include "shape_match/overlay/MatchDebugOverlayAdapter.h"

#include <chrono>
#include <filesystem>
#include <iostream>

namespace ShapeMatch {

namespace {

bool expect(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "[ShapeMatchEval][FAIL] " << message << '\n';
    }
    return condition;
}

void printCandidate(const ScoredCandidate& c)
{
    std::cout << "\n[Candidate #" << c.rank << "]\n"
              << "pose = x=" << c.pose.x << ", y=" << c.pose.y
              << ", theta=" << c.pose.thetaDeg() << " deg, scale=" << c.pose.scale << '\n'
              << "finalScore = " << c.score.finalScore << '\n'
              << "coverage = " << c.score.coverageRatio << '\n'
              << "inlierRatio = " << c.score.inlierRatio << '\n'
              << "rms = " << c.score.rmsError << " px\n"
              << "median = " << c.score.medianError << " px\n"
              << "p90 = " << c.score.p90Error << " px\n"
              << "orientationScore = " << c.score.orientationScore << '\n'
              << "polarityScore = " << c.score.polarityScore << '\n'
              << "valid/found/inlier = " << c.score.validPoints << " / "
              << c.score.foundPoints << " / " << c.score.inlierPoints << '\n'
              << "accepted = " << (c.score.accepted ? "true" : "false") << '\n';
    if (!c.score.accepted) {
        std::cout << "rejectReason = " << c.score.rejectReason << '\n';
    }
    if (c.poseError.hasGroundTruth) {
        std::cout << "poseError = dx=" << c.poseError.dx
                  << " dy=" << c.poseError.dy
                  << " dxy=" << c.poseError.dxy
                  << " dtheta=" << c.poseError.dthetaDeg
                  << " poseOk=" << (c.poseError.poseOk ? "true" : "false") << '\n';
    }
}

MatchDiagnosticReport makeReport(const ShapeTemplateModel& model,
                                 const std::vector<ScoredCandidate>& ranked,
                                 const std::vector<GroundTruthInstance>& gt,
                                 const MultiTargetEvalResult& multi,
                                 double evalTimeMs)
{
    MatchDiagnosticReport report;
    report.imageName = "synthetic_rectangle_image";
    report.templateName = model.templateId;
    report.candidateCount = 6;
    report.topCandidates = ranked;
    report.groundTruthInstances = gt;
    report.multiTargetEval = multi;
    report.evaluationTimeMs = evalTimeMs;
    report.totalTimeMs = evalTimeMs;
    return report;
}

ShapeTemplateModel reversedPolarityCopy(ShapeTemplateModel model)
{
    for (TemplatePoint& p : model.points) {
        p.polarity = EdgePolarity::BrightToDark;
    }
    return model;
}

} // namespace

bool runShapeMatchEvalSelfTest()
{
    ShapeMatchEvalConfig config;
    config.topK = 10;

    const ShapeTemplateModel model = ShapeTemplateModel::createSyntheticRectangle("synthetic_rectangle", 120.0, 80.0, 60);
    const MatchPose groundTruthPose = MatchPose::fromDeg(438.2, 291.6, 13.5, 1.0);
    const std::vector<GroundTruthInstance> gt{{"obj_001", groundTruthPose}};
    const EdgeImageData edgeData = EdgeImageData::createFromTemplateAndPose(model, groundTruthPose, cv::Size(900, 700));
    const std::vector<MatchPose> candidates{
        groundTruthPose,
        MatchPose::fromDeg(448.2, 301.6, 13.5, 1.0),
        MatchPose::fromDeg(438.2, 291.6, 23.5, 1.0),
        MatchPose::fromDeg(438.2, 291.6, 13.5, 1.08),
        MatchPose::fromDeg(520.0, 180.0, -30.0, 0.9),
        MatchPose::fromDeg(438.2, 291.6, -346.5, 1.0)
    };

    const auto t0 = std::chrono::steady_clock::now();
    CandidateRanker ranker(config);
    std::vector<ScoredCandidate> ranked = ranker.rank(candidates, model, edgeData, &gt);
    MultiTargetEvaluator multiEvaluator(config);
    const MultiTargetEvalResult multi = multiEvaluator.evaluate(ranked, gt);
    const auto t1 = std::chrono::steady_clock::now();
    const double evalTimeMs = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::cout << "[ShapeMatchEval] candidate_count=" << candidates.size()
              << " topK=" << config.topK
              << " eval_time=" << evalTimeMs << "ms\n";
    for (const ScoredCandidate& c : ranked) {
        printCandidate(c);
    }

    bool ok = true;
    ok &= expect(!ranked.empty(), "ranked candidates should not be empty");
    const ScoredCandidate& top = ranked.front();
    ok &= expect(top.poseError.poseOk, "correct pose should be top-ranked and poseOk");
    ok &= expect(top.score.finalScore > 0.90, "correct pose finalScore should be high");
    ok &= expect(top.score.coverageRatio > 0.95, "correct pose coverage should be high");
    ok &= expect(top.score.inlierRatio > 0.95, "correct pose inlier ratio should be high");
    ok &= expect(top.score.rmsError < 0.75, "correct pose RMS should be low");

    MatchEvaluator evaluator(config);
    std::vector<PointEval> correctEvals;
    const MatchScore correctScore = evaluator.evaluate(model, edgeData, groundTruthPose, &correctEvals);
    const MatchScore translatedScore = evaluator.evaluate(model, edgeData, candidates[1], nullptr);
    const MatchScore angleScore = evaluator.evaluate(model, edgeData, candidates[2], nullptr);
    const ShapeTemplateModel wrongPolarityModel = reversedPolarityCopy(model);
    const MatchScore wrongPolarityScore = evaluator.evaluate(wrongPolarityModel, edgeData, groundTruthPose, nullptr);

    ok &= expect(translatedScore.distanceScore < correctScore.distanceScore, "translated pose distanceScore should drop");
    ok &= expect(translatedScore.inlierRatio < correctScore.inlierRatio, "translated pose inlierRatio should drop");
    ok &= expect(translatedScore.rmsError > correctScore.rmsError, "translated pose RMS should increase");
    ok &= expect(angleScore.orientationScore < correctScore.orientationScore, "angle error orientationScore should drop");

    PoseErrorEvaluator poseEvaluator(config);
    const ContourReprojectionError angleContour =
        poseEvaluator.evaluateContourReprojectionError(model, candidates[2], groundTruthPose);
    ok &= expect(angleContour.rmsError > 1.0, "angle error contour reprojection error should increase");
    ok &= expect(wrongPolarityScore.polarityScore < correctScore.polarityScore, "wrong polarity should lower polarityScore");
    ok &= expect(wrongPolarityScore.finalScore < correctScore.finalScore, "wrong polarity should lower finalScore");

    const PoseError wrapError = poseEvaluator.evaluatePoseError(MatchPose::fromDeg(0.0, 0.0, 179.0, 1.0),
                                                                MatchPose::fromDeg(0.0, 0.0, -179.0, 1.0));
    ok &= expect(std::abs(wrapError.dthetaDeg - 2.0) < 1e-6, "theta wrap should report 2 deg, not 358 deg");

    std::vector<ScoredCandidate> manualTopK(5);
    manualTopK[0].rank = 1;
    manualTopK[0].pose = MatchPose::fromDeg(100.0, 100.0, 0.0, 1.0);
    manualTopK[1].rank = 2;
    manualTopK[1].pose = MatchPose::fromDeg(200.0, 200.0, 0.0, 1.0);
    manualTopK[2].rank = 3;
    manualTopK[2].pose = groundTruthPose;
    manualTopK[3].rank = 4;
    manualTopK[3].pose = MatchPose::fromDeg(300.0, 200.0, 0.0, 1.0);
    manualTopK[4].rank = 5;
    manualTopK[4].pose = MatchPose::fromDeg(400.0, 100.0, 0.0, 1.0);
    const MultiTargetEvalResult manualHit = multiEvaluator.evaluate(manualTopK, gt);
    ok &= expect(!manualHit.top1Hit && manualHit.top3Hit && manualHit.top5Hit, "top-K hit should detect correct pose at rank 3");

    MatchDiagnosticReport report = makeReport(model, ranked, gt, multi, evalTimeMs);
    MatchReportWriter writer;
    const std::filesystem::path reportsDir = std::filesystem::path("data") / "shape_match" / "reports";
    ok &= expect(writer.writeAll(report, reportsDir), "report files should be generated");
    ok &= expect(std::filesystem::exists(reportsDir / "latest_match_report.json"), "latest_match_report.json should exist");
    ok &= expect(std::filesystem::exists(reportsDir / "latest_candidates.csv"), "latest_candidates.csv should exist");
    ok &= expect(std::filesystem::exists(reportsDir / "latest_point_eval.csv"), "latest_point_eval.csv should exist");
    ok &= expect(std::filesystem::exists(reportsDir / "latest_summary.txt"), "latest_summary.txt should exist");

    MatchDebugOverlayAdapter overlayAdapter;
    const ShapeMatchOverlayData overlay = overlayAdapter.buildOverlayData(report, model);
    ok &= expect(!overlay.empty(), "overlay data should be generated");
    std::cout << "\n[ShapeMatchEval] overlay polylines=" << overlay.polylines.size()
              << " points=" << overlay.points.size()
              << " lines=" << overlay.lines.size()
              << " texts=" << overlay.texts.size() << '\n';
    std::cout << "[ShapeMatchEval] reports_dir=" << reportsDir.string() << '\n';

    if (ok) {
        std::cout << "[ShapeMatchEval] self-test PASSED\n";
    }
    return ok;
}

} // namespace ShapeMatch
