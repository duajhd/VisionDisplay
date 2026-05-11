#include "VisionDisplay/GraphicManager.h"

#include <algorithm>

namespace VisionDisplay {

void GraphicManager::addGraphic(const GraphicObject& graphic)
{
    const auto existing = std::find_if(m_graphics.begin(),
                                       m_graphics.end(),
                                       [&graphic](const GraphicObject& item) {
                                           return item.id == graphic.id;
                                       });
    if (existing != m_graphics.end()) {
        *existing = graphic;
    } else {
        m_graphics.append(graphic);
    }

    sortByLayer();
}

bool GraphicManager::removeGraphic(const QString& id)
{
    const auto oldSize = m_graphics.size();
    m_graphics.erase(std::remove_if(m_graphics.begin(),
                                    m_graphics.end(),
                                    [&id](const GraphicObject& item) {
                                        return item.id == id;
                                    }),
                     m_graphics.end());
    return m_graphics.size() != oldSize;
}

void GraphicManager::clear()
{
    m_graphics.clear();
}

void GraphicManager::clearLayer(int layer)
{
    m_graphics.erase(std::remove_if(m_graphics.begin(),
                                    m_graphics.end(),
                                    [layer](const GraphicObject& item) {
                                        return item.layer == layer;
                                    }),
                     m_graphics.end());
}

void GraphicManager::clearResultGraphics()
{
    m_graphics.erase(std::remove_if(m_graphics.begin(),
                                    m_graphics.end(),
                                    [](const GraphicObject& item) {
                                        return item.resultGraphic;
                                    }),
                     m_graphics.end());
}

void GraphicManager::clearToolGraphics(const QString& toolId)
{
    m_graphics.erase(std::remove_if(m_graphics.begin(),
                                    m_graphics.end(),
                                    [&toolId](const GraphicObject& item) {
                                        return item.toolGraphic && item.toolId == toolId;
                                    }),
                     m_graphics.end());
    m_toolVisibility.remove(toolId);
}

void GraphicManager::clearAllToolGraphics()
{
    m_graphics.erase(std::remove_if(m_graphics.begin(),
                                    m_graphics.end(),
                                    [](const GraphicObject& item) {
                                        return item.toolGraphic;
                                    }),
                     m_graphics.end());
    m_toolVisibility.clear();
}

void GraphicManager::setLayerVisible(int layer, bool visible)
{
    m_layerVisibility.insert(layer, visible);
}

bool GraphicManager::isLayerVisible(int layer) const
{
    return m_layerVisibility.value(layer, true);
}

void GraphicManager::setToolGraphicsVisible(const QString& toolId, bool visible)
{
    m_toolVisibility.insert(toolId, visible);
}

bool GraphicManager::isToolGraphicsVisible(const QString& toolId) const
{
    return m_toolVisibility.value(toolId, true);
}

bool GraphicManager::hasToolGraphics(const QString& toolId) const
{
    return std::any_of(m_graphics.begin(), m_graphics.end(), [&toolId](const GraphicObject& item) {
        return item.toolGraphic && item.toolId == toolId;
    });
}

int GraphicManager::toolGraphicsCount(const QString& toolId) const
{
    return static_cast<int>(std::count_if(m_graphics.begin(), m_graphics.end(), [&toolId](const GraphicObject& item) {
        return item.toolGraphic && item.toolId == toolId;
    }));
}

GraphicObject* GraphicManager::findById(const QString& id)
{
    const auto existing = std::find_if(m_graphics.begin(),
                                       m_graphics.end(),
                                       [&id](const GraphicObject& item) {
                                           return item.id == id;
                                       });
    return existing == m_graphics.end() ? nullptr : &(*existing);
}

const GraphicObject* GraphicManager::findById(const QString& id) const
{
    const auto existing = std::find_if(m_graphics.begin(),
                                       m_graphics.end(),
                                       [&id](const GraphicObject& item) {
                                           return item.id == id;
                                       });
    return existing == m_graphics.end() ? nullptr : &(*existing);
}

const QList<GraphicObject>& GraphicManager::graphics() const
{
    return m_graphics;
}

void GraphicManager::sortByLayer()
{
    std::stable_sort(m_graphics.begin(),
                     m_graphics.end(),
                     [](const GraphicObject& lhs, const GraphicObject& rhs) {
                         return lhs.layer < rhs.layer;
                     });
}

} // namespace VisionDisplay
