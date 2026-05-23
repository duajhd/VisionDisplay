#include "VisionTools/Matching/ShapeTemplateModel.h"

namespace VisionTools::Matching {

bool ShapeTemplateModel::isValid() const
{
    return totalPointCount() > 0;
}

int ShapeTemplateModel::totalPointCount() const
{
    int count = 0;
    for (const TemplateLevel& level : levels) {
        count += static_cast<int>(level.points.size());
    }
    return count;
}

} // namespace VisionTools::Matching
