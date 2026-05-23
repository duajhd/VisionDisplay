#pragma once

#include "shape_match/coarse/VotingConfig.h"
#include "shape_match/core/ShapeTemplateModel.h"

#include <cstdint>
#include <vector>

namespace ShapeMatch {

struct RTEntry
{
    float dx = 0.0f;
    float dy = 0.0f;
    float nx = 1.0f;
    float ny = 0.0f;
    float weight = 1.0f;
    uint8_t polarity = 0;
    int pointId = -1;
    int thetaBin = 0;
    int templateGradBin = 0;
};

class RTTable
{
public:
    bool build(const ShapeTemplateModel& model, const VotingConfig& config);

    const std::vector<RTEntry>& entriesForImageGradientBin(int imageGradBin) const;

    int orientationBinCount() const;
    int totalEntryCount() const;
    int selectedTemplatePointCount() const;

private:
    static int angleToBin(double angleRad, int binCount);
    static uint8_t polarityToByte(EdgePolarity polarity);

    std::vector<const TemplatePoint*> selectTemplatePoints(const ShapeTemplateModel& model,
                                                           const VotingConfig& config) const;

    VotingConfig m_config;
    std::vector<std::vector<RTEntry>> m_entriesByImageGradBin;
    int m_totalEntryCount = 0;
    int m_selectedTemplatePointCount = 0;
};

} // namespace ShapeMatch
