#pragma once

namespace ShapeMatch {

enum class MicroAdjustMode {
    Disabled,
    Minimal,
    Recall
};

inline const char* microAdjustModeName(MicroAdjustMode mode)
{
    switch (mode) {
    case MicroAdjustMode::Disabled:
        return "Disabled";
    case MicroAdjustMode::Minimal:
        return "Minimal";
    case MicroAdjustMode::Recall:
        return "Recall";
    }
    return "Unknown";
}

struct MicroAdjustProfile
{
    bool enabled = false;
    MicroAdjustMode mode = MicroAdjustMode::Minimal;

    int inputCandidateCount = 0;
    int adjustedCandidateCount = 0;

    int generatedPoseCount = 0;
    int scoredPoseCount = 0;
    int keptPoseCount = 0;

    int maxChildrenPerCandidate = 0;
    int translationSampleCount = 0;
    int angleSampleCount = 0;

    double generateMs = 0.0;
    double scoreMs = 0.0;
    double nmsMs = 0.0;
    double totalMs = 0.0;

    int prunedCount = 0;
    int nmsSuppressedCount = 0;
};

} // namespace ShapeMatch
