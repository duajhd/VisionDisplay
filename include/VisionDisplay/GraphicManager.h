#pragma once

#include "VisionDisplay/GraphicObject.h"
#include "VisionDisplay/VisionDisplay_global.h"

#include <QHash>
#include <QList>
#include <QString>

namespace VisionDisplay {

class VISIONDISPLAY_API GraphicManager
{
public:
    void addGraphic(const GraphicObject& graphic);
    bool removeGraphic(const QString& id);
    void clear();
    void clearLayer(int layer);
    void clearResultGraphics();
    void clearToolGraphics(const QString& toolId);
    void clearAllToolGraphics();

    void setLayerVisible(int layer, bool visible);
    bool isLayerVisible(int layer) const;
    void setToolGraphicsVisible(const QString& toolId, bool visible);
    bool isToolGraphicsVisible(const QString& toolId) const;
    bool hasToolGraphics(const QString& toolId) const;
    int toolGraphicsCount(const QString& toolId) const;

    GraphicObject* findById(const QString& id);
    const GraphicObject* findById(const QString& id) const;
    const QList<GraphicObject>& graphics() const;

private:
    void sortByLayer();

    QList<GraphicObject> m_graphics;
    QHash<int, bool> m_layerVisibility;
    QHash<QString, bool> m_toolVisibility;
};

} // namespace VisionDisplay
