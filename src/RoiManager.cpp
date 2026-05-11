#include "VisionDisplay/RoiManager.h"

#include <algorithm>

namespace VisionDisplay {

void RoiManager::addRoi(std::unique_ptr<RoiObject> roi)
{
    if (!roi) {
        return;
    }

    removeRoi(roi->id);
    m_selectedId = roi->id;
    roi->selected = true;
    m_rois.push_back(std::move(roi));
}

bool RoiManager::removeRoi(const QString& id)
{
    const auto oldSize = m_rois.size();
    m_rois.erase(std::remove_if(m_rois.begin(),
                                m_rois.end(),
                                [&id](const std::unique_ptr<RoiObject>& roi) {
                                    return roi && roi->id == id;
                                }),
                 m_rois.end());
    if (m_rois.size() != oldSize) {
        if (m_selectedId == id) {
            m_selectedId.clear();
        }
        return true;
    }
    return false;
}

bool RoiManager::removeSelectedRoi()
{
    if (m_selectedId.isEmpty()) {
        return false;
    }
    return removeRoi(m_selectedId);
}

void RoiManager::clear()
{
    m_rois.clear();
    m_selectedId.clear();
}

RoiObject* RoiManager::findById(const QString& id)
{
    const auto found = std::find_if(m_rois.begin(),
                                    m_rois.end(),
                                    [&id](const std::unique_ptr<RoiObject>& roi) {
                                        return roi && roi->id == id;
                                    });
    return found == m_rois.end() ? nullptr : found->get();
}

const RoiObject* RoiManager::findById(const QString& id) const
{
    const auto found = std::find_if(m_rois.begin(),
                                    m_rois.end(),
                                    [&id](const std::unique_ptr<RoiObject>& roi) {
                                        return roi && roi->id == id;
                                    });
    return found == m_rois.end() ? nullptr : found->get();
}

RoiObject* RoiManager::selectedRoi() const
{
    return const_cast<RoiObject*>(findById(m_selectedId));
}

QString RoiManager::selectedRoiId() const
{
    return m_selectedId;
}

bool RoiManager::selectRoi(const QString& id)
{
    bool changed = m_selectedId != id;
    bool found = false;

    for (const std::unique_ptr<RoiObject>& roi : m_rois) {
        if (!roi) {
            continue;
        }
        const bool selected = roi->id == id;
        roi->selected = selected;
        found = found || selected;
    }

    if (found) {
        m_selectedId = id;
    } else {
        changed = changed || !m_selectedId.isEmpty();
        m_selectedId.clear();
    }

    return changed;
}

void RoiManager::clearSelection()
{
    for (const std::unique_ptr<RoiObject>& roi : m_rois) {
        if (roi) {
            roi->selected = false;
        }
    }
    m_selectedId.clear();
}

QString RoiManager::hitTest(const QPointF& imagePoint, double imageTolerance) const
{
    for (auto it = m_rois.rbegin(); it != m_rois.rend(); ++it) {
        const RoiObject* roi = it->get();
        if (roi && roi->visible && roi->hitTest(imagePoint, imageTolerance)) {
            return roi->id;
        }
    }
    return {};
}

const std::vector<std::unique_ptr<RoiObject>>& RoiManager::rois() const
{
    return m_rois;
}

QJsonArray RoiManager::toJson() const
{
    QJsonArray array;
    for (const std::unique_ptr<RoiObject>& roi : m_rois) {
        if (roi) {
            array.append(roi->toJson());
        }
    }
    return array;
}

} // namespace VisionDisplay
