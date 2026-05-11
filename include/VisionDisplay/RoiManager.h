#pragma once

#include "VisionDisplay/RoiObject.h"
#include "VisionDisplay/VisionDisplay_global.h"

#include <QJsonArray>
#include <QString>

#include <memory>
#include <vector>

namespace VisionDisplay {

class VISIONDISPLAY_API RoiManager
{
public:
    RoiManager() = default;
    ~RoiManager() = default;

    RoiManager(const RoiManager&) = delete;
    RoiManager& operator=(const RoiManager&) = delete;
    RoiManager(RoiManager&&) noexcept = default;
    RoiManager& operator=(RoiManager&&) noexcept = default;

    void addRoi(std::unique_ptr<RoiObject> roi);
    bool removeRoi(const QString& id);
    bool removeSelectedRoi();
    void clear();

    RoiObject* findById(const QString& id);
    const RoiObject* findById(const QString& id) const;
    RoiObject* selectedRoi() const;
    QString selectedRoiId() const;

    bool selectRoi(const QString& id);
    void clearSelection();
    QString hitTest(const QPointF& imagePoint, double imageTolerance) const;

    const std::vector<std::unique_ptr<RoiObject>>& rois() const;
    QJsonArray toJson() const;

private:
    std::vector<std::unique_ptr<RoiObject>> m_rois;
    QString m_selectedId;
};

} // namespace VisionDisplay
