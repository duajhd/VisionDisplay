#include "shape_match/io/VisionProGroundTruthReader.h"

#include <filesystem>
#include <fstream>
#include <iostream>

int main()
{
    const std::filesystem::path path = std::filesystem::path("data") / "shape_match" / "reports" / "visionpro_gt_reader_selftest.json";
    std::filesystem::create_directories(path.parent_path());
    {
        std::ofstream file(path);
        file << ShapeMatch::VisionProGroundTruthReader::exampleJson();
    }

    ShapeMatch::VisionProGroundTruthReader reader;
    const ShapeMatch::VisionProGroundTruthReadResult result = reader.read(path);
    if (!result.ok()) {
        std::cerr << result.error << "\n";
        return 1;
    }
    if (result.instances.size() != 1) {
        std::cerr << "unexpected instance count\n";
        return 2;
    }
    if (!result.training.hasTrainingRoi || !result.training.hasOrigin
        || std::abs(result.training.roiX - 120.0) > 1e-9
        || std::abs(result.training.originX - 230.0) > 1e-9) {
        std::cerr << "unexpected training metadata\n";
        return 3;
    }
    const ShapeMatch::GroundTruthInstance& gt = result.instances.front();
    if (gt.id != "obj_001" || std::abs(gt.pose.x - 438.2) > 1e-9
        || std::abs(gt.pose.y - 291.6) > 1e-9
        || std::abs(gt.pose.thetaDeg() - 13.5) > 1e-9) {
        std::cerr << "unexpected parsed pose\n";
        return 4;
    }

    std::cout << "VisionProGroundTruthReaderSelfTest passed\n";
    return 0;
}
