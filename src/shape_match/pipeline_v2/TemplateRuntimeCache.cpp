#include "shape_match/pipeline_v2/TemplateRuntimeCache.h"

#include <sstream>

namespace ShapeMatch {

std::string TemplateRuntimeCache::responseKey(const ShapeTemplateModel& model, int level, const OrientationResponseConfig& config)
{
    std::ostringstream oss;
    oss << model.templateId << ':' << model.points.size() << ':' << level << ':'
        << config.orientationBinCount << ':' << config.maxTemplateResponsePoints;
    return oss.str();
}

std::string TemplateRuntimeCache::segmentKey(const ShapeTemplateModel& model, int level, const DirectionalChamferConfig& config)
{
    std::ostringstream oss;
    oss << model.templateId << ':' << model.points.size() << ':' << level << ':'
        << config.orientationBinCount << ':' << config.maxTemplateVerifyPoints << ':' << config.maxSegments;
    return oss.str();
}

const ResponseTemplate& TemplateRuntimeCache::getOrBuildResponseTemplate(const ShapeTemplateModel& model,
                                                                         int level,
                                                                         const OrientationResponseConfig& config)
{
    const std::string key = responseKey(model, level, config);
    std::lock_guard<std::mutex> lock(m_mutex);
    auto it = m_responseTemplates.find(key);
    if (it != m_responseTemplates.end()) {
        ++m_responseHits;
        return it->second;
    }
    ++m_responseMisses;
    ResponseTemplateBuilder builder;
    auto inserted = m_responseTemplates.emplace(key, builder.build(model, level, config));
    return inserted.first->second;
}

const SegmentTemplate& TemplateRuntimeCache::getOrBuildSegmentTemplate(const ShapeTemplateModel& model,
                                                                       int level,
                                                                       const DirectionalChamferConfig& config)
{
    const std::string key = segmentKey(model, level, config);
    std::lock_guard<std::mutex> lock(m_mutex);
    auto it = m_segmentTemplates.find(key);
    if (it != m_segmentTemplates.end()) {
        ++m_segmentHits;
        return it->second;
    }
    ++m_segmentMisses;
    SegmentTemplateBuilder builder;
    auto inserted = m_segmentTemplates.emplace(key, builder.build(model, level, config));
    return inserted.first->second;
}

void TemplateRuntimeCache::clear()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_responseTemplates.clear();
    m_segmentTemplates.clear();
}

int TemplateRuntimeCache::responseTemplateHitCount() const { return m_responseHits; }
int TemplateRuntimeCache::responseTemplateMissCount() const { return m_responseMisses; }
int TemplateRuntimeCache::segmentTemplateHitCount() const { return m_segmentHits; }
int TemplateRuntimeCache::segmentTemplateMissCount() const { return m_segmentMisses; }

} // namespace ShapeMatch
