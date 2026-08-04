#pragma once

#include "shape_match/coarse/ResponseTemplate.h"
#include "shape_match/pipeline_v2/SegmentTemplate.h"

#include <map>
#include <mutex>
#include <string>

namespace ShapeMatch {

class TemplateRuntimeCache
{
public:
    const ResponseTemplate& getOrBuildResponseTemplate(const ShapeTemplateModel& model,
                                                       int level,
                                                       const OrientationResponseConfig& config);

    const SegmentTemplate& getOrBuildSegmentTemplate(const ShapeTemplateModel& model,
                                                     int level,
                                                     const DirectionalChamferConfig& config);

    void clear();

    int responseTemplateHitCount() const;
    int responseTemplateMissCount() const;
    int segmentTemplateHitCount() const;
    int segmentTemplateMissCount() const;

private:
    static std::string responseKey(const ShapeTemplateModel& model, int level, const OrientationResponseConfig& config);
    static std::string segmentKey(const ShapeTemplateModel& model, int level, const DirectionalChamferConfig& config);

    mutable std::mutex m_mutex;
    std::map<std::string, ResponseTemplate> m_responseTemplates;
    std::map<std::string, SegmentTemplate> m_segmentTemplates;
    int m_responseHits = 0;
    int m_responseMisses = 0;
    int m_segmentHits = 0;
    int m_segmentMisses = 0;
};

} // namespace ShapeMatch
