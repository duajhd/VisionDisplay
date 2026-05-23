#include "shape_match/io/VisionProGroundTruthReader.h"

#include <nlohmann/json.hpp>

#include <fstream>
#include <sstream>

namespace ShapeMatch {

namespace {

double numberOr(const nlohmann::json& object, const char* key, double fallback)
{
    const auto it = object.find(key);
    return it != object.end() && it->is_number() ? it->get<double>() : fallback;
}

std::string stringOr(const nlohmann::json& object, const char* key, const std::string& fallback)
{
    const auto it = object.find(key);
    return it != object.end() && it->is_string() ? it->get<std::string>() : fallback;
}

bool boolOr(const nlohmann::json& object, const char* key, bool fallback)
{
    const auto it = object.find(key);
    return it != object.end() && it->is_boolean() ? it->get<bool>() : fallback;
}

} // namespace

const char* VisionProGroundTruthReader::schemaName()
{
    return "visionpro_cogpmalign_ground_truth_v1";
}

std::string VisionProGroundTruthReader::exampleJson()
{
    return R"json({
  "schema": "visionpro_cogpmalign_ground_truth_v1",
  "image": "part_001.png",
  "template": "shape_model_roi",
  "source": "visionpro_cogpmalign",
  "training": {
    "roi": { "x": 120.0, "y": 80.0, "width": 220.0, "height": 160.0 },
    "origin": { "x": 230.0, "y": 160.0, "kind": "template_origin" }
  },
  "pose_convention": {
    "origin": "template_origin",
    "angle_unit": "deg",
    "coordinate_system": "image_xy_y_down",
    "positive_angle": "same_as_match_pose"
  },
  "instances": [
    {
      "id": "obj_001",
      "x": 438.2,
      "y": 291.6,
      "theta_deg": 13.5,
      "scale": 1.0,
      "score": 0.93,
      "accepted": true
    }
  ]
})json";
}

VisionProGroundTruthReadResult VisionProGroundTruthReader::read(const std::filesystem::path& path) const
{
    VisionProGroundTruthReadResult result;
    std::ifstream file(path);
    if (!file) {
        result.error = "failed_to_open_ground_truth_json: " + path.string();
        return result;
    }

    nlohmann::json root;
    try {
        file >> root;
    } catch (const std::exception& e) {
        result.error = std::string("failed_to_parse_ground_truth_json: ") + e.what();
        return result;
    }

    if (!root.is_object()) {
        result.error = "ground_truth_json_root_must_be_object";
        return result;
    }

    const std::string schema = stringOr(root, "schema", {});
    if (!schema.empty() && schema != schemaName()) {
        result.warnings.push_back("unexpected_schema: " + schema);
    }
    result.imageName = stringOr(root, "image", {});
    result.templateName = stringOr(root, "template", {});
    result.source = stringOr(root, "source", result.source);

    if (const auto trainingIt = root.find("training"); trainingIt != root.end() && trainingIt->is_object()) {
        if (const auto roiIt = trainingIt->find("roi"); roiIt != trainingIt->end() && roiIt->is_object()) {
            result.training.hasTrainingRoi = true;
            result.training.roiX = numberOr(*roiIt, "x", 0.0);
            result.training.roiY = numberOr(*roiIt, "y", 0.0);
            result.training.roiWidth = numberOr(*roiIt, "width", 0.0);
            result.training.roiHeight = numberOr(*roiIt, "height", 0.0);
            if (result.training.roiWidth <= 0.0 || result.training.roiHeight <= 0.0) {
                result.warnings.push_back("training_roi_has_non_positive_size");
            }
        }
        if (const auto originIt = trainingIt->find("origin"); originIt != trainingIt->end() && originIt->is_object()) {
            result.training.hasOrigin = true;
            result.training.originX = numberOr(*originIt, "x", 0.0);
            result.training.originY = numberOr(*originIt, "y", 0.0);
            result.training.originKind = stringOr(*originIt, "kind", result.training.originKind);
        }
    }

    if (const auto poseIt = root.find("pose_convention"); poseIt != root.end() && poseIt->is_object()) {
        result.poseOrigin = stringOr(*poseIt, "origin", result.poseOrigin);
        const std::string angleUnit = stringOr(*poseIt, "angle_unit", "deg");
        if (angleUnit != "deg") {
            result.error = "unsupported_angle_unit: " + angleUnit + " (expected deg)";
            return result;
        }
        const std::string coordinateSystem = stringOr(*poseIt, "coordinate_system", "image_xy_y_down");
        if (coordinateSystem != "image_xy_y_down") {
            result.warnings.push_back("unexpected_coordinate_system: " + coordinateSystem);
        }
    }
    if (result.poseOrigin != "template_origin") {
        result.error = "unsupported_pose_origin: " + result.poseOrigin + " (expected template_origin)";
        return result;
    }
    if (!result.training.hasTrainingRoi) {
        result.warnings.push_back("missing_training_roi");
    }
    if (!result.training.hasOrigin) {
        result.warnings.push_back("missing_training_origin");
    }

    const auto instancesIt = root.find("instances");
    if (instancesIt == root.end() || !instancesIt->is_array()) {
        result.error = "ground_truth_json_missing_instances_array";
        return result;
    }

    int autoId = 0;
    for (const nlohmann::json& item : *instancesIt) {
        if (!item.is_object()) {
            result.warnings.push_back("skipped_non_object_instance");
            continue;
        }
        if (!boolOr(item, "accepted", true)) {
            continue;
        }
        const auto xIt = item.find("x");
        const auto yIt = item.find("y");
        if (xIt == item.end() || yIt == item.end() || !xIt->is_number() || !yIt->is_number()) {
            result.warnings.push_back("skipped_instance_missing_numeric_x_y");
            continue;
        }
        const double thetaDeg = item.contains("theta_deg")
            ? numberOr(item, "theta_deg", 0.0)
            : numberOr(item, "angle_deg", 0.0);
        const double scale = numberOr(item, "scale", 1.0);
        GroundTruthInstance instance;
        instance.id = stringOr(item, "id", "visionpro_" + std::to_string(autoId));
        instance.pose = MatchPose::fromDeg(xIt->get<double>(), yIt->get<double>(), thetaDeg, scale);
        result.instances.push_back(instance);
        ++autoId;
    }

    if (result.instances.empty()) {
        result.error = "ground_truth_json_has_no_accepted_instances";
    }
    return result;
}

} // namespace ShapeMatch
