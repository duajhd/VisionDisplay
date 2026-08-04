#include "shape_match/core/EdgeImageData.h"
#include "shape_match/core/ShapeTemplateModel.h"
#include "shape_match/pipeline_v2/DirectionalChamferVerifier.h"
#include "shape_match/pipeline_v2/DirectionalDistanceFieldBuilder.h"
#include "shape_match/pipeline_v2/SegmentTemplate.h"

#include <algorithm>
#include <iostream>

namespace ShapeMatch {

namespace {

bool expect(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "[DirectionalChamfer][FAIL] " << message << '\n';
    }
    return condition;
}

EdgeImageData emptyData(cv::Size size)
{
    EdgeImageData data;
    data.imageSize = size;
    data.edgeMap = cv::Mat::zeros(size, CV_8U);
    data.gradX = cv::Mat::zeros(size, CV_32F);
    data.gradY = cv::Mat::zeros(size, CV_32F);
    data.gradMag = cv::Mat::zeros(size, CV_32F);
    return data;
}

void addInstance(EdgeImageData& dst, const ShapeTemplateModel& model, const MatchPose& pose, double keepRatio = 1.0)
{
    EdgeImageData instance = EdgeImageData::createFromTemplateAndPose(model, pose, dst.imageSize);
    const size_t keep = static_cast<size_t>(instance.edgePoints.size() * keepRatio);
    for (size_t i = 0; i < keep; ++i) {
        const cv::Point2d p = instance.edgePoints[i];
        const cv::Point2d n = i < instance.edgeNormals.size() ? instance.edgeNormals[i] : cv::Point2d(1.0, 0.0);
        const int x = std::clamp(static_cast<int>(std::round(p.x)), 0, dst.imageSize.width - 1);
        const int y = std::clamp(static_cast<int>(std::round(p.y)), 0, dst.imageSize.height - 1);
        dst.edgePoints.emplace_back(x, y);
        dst.edgeNormals.push_back(n);
        dst.edgeMap.at<uchar>(y, x) = 255;
        dst.gradX.at<float>(y, x) = static_cast<float>(n.x);
        dst.gradY.at<float>(y, x) = static_cast<float>(n.y);
        dst.gradMag.at<float>(y, x) = 255.0f;
    }
}

void addLine(EdgeImageData& data, int x)
{
    for (int y = 20; y < data.imageSize.height - 20; ++y) {
        data.edgePoints.emplace_back(x, y);
        data.edgeNormals.emplace_back(1.0, 0.0);
        data.edgeMap.at<uchar>(y, x) = 255;
        data.gradX.at<float>(y, x) = 1.0f;
        data.gradY.at<float>(y, x) = 0.0f;
        data.gradMag.at<float>(y, x) = 255.0f;
    }
}

} // namespace

int runDirectionalChamferVerifierSelfTest()
{
    bool ok = true;
    DirectionalChamferConfig config;
    config.orientationBinCount = 16;
    config.maxTemplateVerifyPoints = 160;
    config.minTemplateVerifyPoints = 20;
    config.maxCandidatesToVerify = 20;
    config.targetCandidatesAfterVerify = 5;

    {
        EdgeImageData data = emptyData(cv::Size(120, 100));
        addLine(data, 60);
        DirectionalDistanceField field = DirectionalDistanceFieldBuilder().build(data, 0, config);
        ok &= expect(field.valid, "directional field should be valid");
        ok &= expect(static_cast<int>(field.distanceBins.size()) == config.orientationBinCount, "field bin count should match config");
        const float nearDistance = field.distanceBins[8].at<float>(50, 60);
        ok &= expect(nearDistance < 1.0f, "correct direction bin should have low distance at line");
    }

    const ShapeTemplateModel model = ShapeTemplateModel::createSyntheticRectangle("dc_rect", 48.0, 32.0, 18);
    SegmentTemplate templ = SegmentTemplateBuilder().build(model, 0, config);
    ok &= expect(!templ.empty(), "segment template should build");
    ok &= expect(templ.segmentCount() >= 4, "rectangle should have several segments");
    ok &= expect(templ.pointCount() <= config.maxTemplateVerifyPoints, "segment template point cap should hold");

    {
        const MatchPose gt = MatchPose::fromDeg(105.0, 85.0, 0.0);
        EdgeImageData data = EdgeImageData::createFromTemplateAndPose(model, gt, cv::Size(220, 170));
        DirectionalDistanceField field = DirectionalDistanceFieldBuilder().build(data, 0, config);
        DirectionalChamferVerifier verifier(config);
        const DirectionalChamferScore good = verifier.scoreCandidate(gt, templ, field);
        const DirectionalChamferScore bad = verifier.scoreCandidate(MatchPose::fromDeg(150.0, 120.0, 45.0), templ, field);
        std::cout << "[DirectionalChamfer] gtScore=" << good.finalScore << " badScore=" << bad.finalScore << '\n';
        ok &= expect(good.finalScore > bad.finalScore, "GT candidate should score higher than bad pose");
        ok &= expect(good.accepted, "GT candidate should be accepted");
    }

    {
        const MatchPose gt = MatchPose::fromDeg(110.0, 90.0, 0.0);
        EdgeImageData data = emptyData(cv::Size(240, 180));
        addInstance(data, model, gt);
        addLine(data, 30);
        DirectionalDistanceField field = DirectionalDistanceFieldBuilder().build(data, 0, config);
        DirectionalChamferVerifier verifier(config);
        const DirectionalChamferScore trueScore = verifier.scoreCandidate(gt, templ, field);
        const DirectionalChamferScore lineScore = verifier.scoreCandidate(MatchPose::fromDeg(30.0, 90.0, 0.0), templ, field);
        ok &= expect(trueScore.finalScore > lineScore.finalScore, "strong line false pose should be lower than true target");
    }

    {
        const MatchPose gt = MatchPose::fromDeg(105.0, 85.0, 0.0);
        EdgeImageData data = emptyData(cv::Size(220, 170));
        addInstance(data, model, gt, 0.55);
        DirectionalDistanceField field = DirectionalDistanceFieldBuilder().build(data, 0, config);
        DirectionalChamferVerifier verifier(config);
        const DirectionalChamferScore occluded = verifier.scoreCandidate(gt, templ, field);
        ok &= expect(occluded.finalScore > 0.20, "occluded target should retain nonzero score");
        ok &= expect(occluded.activeSegments > 0, "occluded target should keep active segments");
    }

    {
        const MatchPose gt = MatchPose::fromDeg(105.0, 85.0, 0.0);
        EdgeImageData data = EdgeImageData::createFromTemplateAndPose(model, gt, cv::Size(220, 170));
        DirectionalDistanceField field = DirectionalDistanceFieldBuilder().build(data, 0, config);
        DirectionalChamferVerifier verifier(config);
        const DirectionalChamferScore good = verifier.scoreCandidate(gt, templ, field, 0.7);
        const DirectionalChamferScore bad = verifier.scoreCandidate(MatchPose::fromDeg(190.0, 145.0, 80.0), templ, field, 0.7);
        ok &= expect(!good.pruned, "good candidate should not be pruned");
        ok &= expect(bad.pruned || bad.finalScore < good.finalScore, "bad candidate should be pruned or score lower");
    }

    if (ok) {
        std::cout << "[DirectionalChamfer] self-test PASSED\n";
    }
    return ok ? 0 : 1;
}

} // namespace ShapeMatch

int main()
{
    return ShapeMatch::runDirectionalChamferVerifierSelfTest();
}
