#pragma once

#include "shape_match/coarse/OrientationResponseMapBuilder.h"
#include "shape_match/pipeline_v2/DirectionalDistanceFieldBuilder.h"

#include <map>
#include <mutex>
#include <string>

namespace ShapeMatch {

class DirectionalFieldCache
{
public:
    const DirectionalDistanceField& getOrBuildDirectionalField(const EdgeImageData& edgeData,
                                                              int level,
                                                              const DirectionalChamferConfig& config);

    const OrientationResponseMap& getOrBuildOrientationResponseMap(const EdgeImageData& edgeData,
                                                                   int level,
                                                                   const OrientationResponseConfig& config);

    void clearFrameCache();
    void clearAll();

    int directionalFieldHitCount() const;
    int directionalFieldMissCount() const;
    int responseMapHitCount() const;
    int responseMapMissCount() const;

private:
    static std::string edgeKey(const EdgeImageData& edgeData, int level, int bins, int extra);

    mutable std::mutex m_mutex;
    std::map<std::string, DirectionalDistanceField> m_directionalFields;
    std::map<std::string, OrientationResponseMap> m_responseMaps;
    int m_directionalHits = 0;
    int m_directionalMisses = 0;
    int m_responseHits = 0;
    int m_responseMisses = 0;
};

} // namespace ShapeMatch
