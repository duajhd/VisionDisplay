#include "shape_match/pipeline_v2/DirectionalFieldCache.h"

#include <sstream>

namespace ShapeMatch {

std::string DirectionalFieldCache::edgeKey(const EdgeImageData& edgeData, int level, int bins, int extra)
{
    std::ostringstream oss;
    oss << static_cast<const void*>(&edgeData) << ':' << edgeData.imageSize.width << 'x' << edgeData.imageSize.height
        << ':' << edgeData.edgePoints.size() << ':' << level << ':' << bins << ':' << extra;
    return oss.str();
}

const DirectionalDistanceField& DirectionalFieldCache::getOrBuildDirectionalField(const EdgeImageData& edgeData,
                                                                                 int level,
                                                                                 const DirectionalChamferConfig& config)
{
    const std::string key = edgeKey(edgeData, level, config.orientationBinCount, config.orientationSpreadRadiusBins);
    std::lock_guard<std::mutex> lock(m_mutex);
    auto it = m_directionalFields.find(key);
    if (it != m_directionalFields.end()) {
        ++m_directionalHits;
        return it->second;
    }
    ++m_directionalMisses;
    DirectionalDistanceFieldBuilder builder;
    auto inserted = m_directionalFields.emplace(key, builder.build(edgeData, level, config));
    return inserted.first->second;
}

const OrientationResponseMap& DirectionalFieldCache::getOrBuildOrientationResponseMap(const EdgeImageData& edgeData,
                                                                                      int level,
                                                                                      const OrientationResponseConfig& config)
{
    const std::string key = edgeKey(edgeData, level, config.orientationBinCount, config.spatialSpreadRadiusPx);
    std::lock_guard<std::mutex> lock(m_mutex);
    auto it = m_responseMaps.find(key);
    if (it != m_responseMaps.end()) {
        ++m_responseHits;
        return it->second;
    }
    ++m_responseMisses;
    OrientationResponseMapBuilder builder;
    auto inserted = m_responseMaps.emplace(key, builder.build(edgeData, level, config));
    return inserted.first->second;
}

void DirectionalFieldCache::clearFrameCache()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_directionalFields.clear();
    m_responseMaps.clear();
}

void DirectionalFieldCache::clearAll()
{
    clearFrameCache();
}

int DirectionalFieldCache::directionalFieldHitCount() const { return m_directionalHits; }
int DirectionalFieldCache::directionalFieldMissCount() const { return m_directionalMisses; }
int DirectionalFieldCache::responseMapHitCount() const { return m_responseHits; }
int DirectionalFieldCache::responseMapMissCount() const { return m_responseMisses; }

} // namespace ShapeMatch
