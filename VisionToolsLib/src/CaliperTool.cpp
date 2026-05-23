#include "VisionTools/CaliperTool.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace VisionTools {

namespace {

constexpr double pi = 3.14159265358979323846;

double clampDouble(double value, double low, double high)
{
    return std::max(low, std::min(high, value));
}

CaliperParams normalizedParams(CaliperParams params)
{
    params.sampleCount = std::max(3, params.sampleCount);
    if (params.sampleCount % 2 == 0) {
        ++params.sampleCount;
    }
    params.projectionCount = std::max(1, params.projectionCount);
    params.minResponse = std::max(0.0, params.minResponse);
    params.maxPositionDeviation = std::max(0.0, params.maxPositionDeviation);
    return params;
}

double responseAt(double gradient, EdgePolarity polarity)
{
    switch (polarity) {
    case EdgePolarity::DarkToLight:
        return gradient;
    case EdgePolarity::LightToDark:
        return -gradient;
    case EdgePolarity::Any:
        return std::abs(gradient);
    }
    return 0.0;
}

std::vector<EdgePoint> singleEdgeResult(const EdgePoint& edge)
{
    std::vector<EdgePoint> result;
    result.reserve(1);
    result.push_back(edge);
    return result;
}

} // namespace

CaliperTool::CaliperTool() = default;

void CaliperTool::setParams(const CaliperParams& params)
{
    m_params = normalizedParams(params);
}

const CaliperParams& CaliperTool::params() const
{
    return m_params;
}

CaliperResult CaliperTool::run(const ImageView& image, const CaliperRegion& region) const
{
    CaliperResult result;
    if (!image.isValid()) {
        result.message = QStringLiteral("Invalid image");
        return result;
    }
    if (region.length <= 0.0 || region.width < 0.0) {
        result.message = QStringLiteral("Invalid caliper region");
        return result;
    }

    bool hadInvalidSamples = false;
    result.profile = sampleProfile(image, region, &hadInvalidSamples);
    result.smoothedProfile = smoothProfile(result.profile);
    result.gradient = computeGradient(result.smoothedProfile);
    result.candidates = detectCandidates(result.gradient, region);
    result.selectedEdges = selectEdges(result.candidates, &result.selectedByFallback);
    result.ok = !result.selectedEdges.empty();

    if (result.ok) {
        result.message = hadInvalidSamples
            ? QStringLiteral("OK; some samples were outside the image")
            : QStringLiteral("OK");
    } else {
        if (result.candidates.empty()) {
            result.message = QStringLiteral("No edge candidate found");
        } else if (m_params.selection == EdgeSelection::NearestToExpected) {
            result.message = QStringLiteral("No edge selected inside expected window");
        } else {
            result.message = QStringLiteral("No edge selected");
        }
        if (hadInvalidSamples) {
            result.message += QStringLiteral("; some samples were outside the image");
        }
    }
    return result;
}

std::vector<double> CaliperTool::sampleProfile(const ImageView& image,
                                               const CaliperRegion& region,
                                               bool* hadInvalidSamples) const
{
    const CaliperParams p = normalizedParams(m_params);
    std::vector<double> profile(static_cast<size_t>(p.sampleCount), 0.0);
    const double angleRad = region.angleDeg * pi / 180.0;
    const double dirX = std::cos(angleRad);
    const double dirY = std::sin(angleRad);
    const double tanX = -dirY;
    const double tanY = dirX;
    const double sampleStep = region.length / static_cast<double>(p.sampleCount - 1);
    const double widthStep = p.projectionCount > 1
        ? region.width / static_cast<double>(p.projectionCount - 1)
        : 0.0;

    bool invalid = false;
    for (int i = 0; i < p.sampleCount; ++i) {
        const double s = -region.length * 0.5 + i * sampleStep;
        double sum = 0.0;
        int validCount = 0;

        for (int j = 0; j < p.projectionCount; ++j) {
            const double t = p.projectionCount > 1 ? -region.width * 0.5 + j * widthStep : 0.0;
            const double x = region.centerX + s * dirX + t * tanX;
            const double y = region.centerY + s * dirY + t * tanY;
            bool ok = false;
            const double value = image.sampleBilinear(x, y, &ok);
            if (ok) {
                sum += value;
                ++validCount;
            } else {
                invalid = true;
            }
        }

        if (validCount > 0) {
            profile[static_cast<size_t>(i)] = sum / validCount;
        } else if (i > 0) {
            profile[static_cast<size_t>(i)] = profile[static_cast<size_t>(i - 1)];
        } else {
            profile[static_cast<size_t>(i)] = 0.0;
        }
    }

    if (hadInvalidSamples) {
        *hadInvalidSamples = invalid;
    }
    return profile;
}

std::vector<double> CaliperTool::smoothProfile(const std::vector<double>& profile) const
{
    const double sigma = m_params.smoothingSigma;
    if (profile.empty() || sigma <= 0.0) {
        return profile;
    }

    const int radius = std::max(1, static_cast<int>(std::ceil(3.0 * sigma)));
    std::vector<double> kernel(static_cast<size_t>(radius * 2 + 1), 0.0);
    double kernelSum = 0.0;
    for (int i = -radius; i <= radius; ++i) {
        const double value = std::exp(-(i * i) / (2.0 * sigma * sigma));
        kernel[static_cast<size_t>(i + radius)] = value;
        kernelSum += value;
    }
    for (double& value : kernel) {
        value /= kernelSum;
    }

    std::vector<double> output(profile.size(), 0.0);
    const int last = static_cast<int>(profile.size()) - 1;
    for (int i = 0; i <= last; ++i) {
        double sum = 0.0;
        for (int k = -radius; k <= radius; ++k) {
            const int index = std::max(0, std::min(last, i + k));
            sum += profile[static_cast<size_t>(index)] * kernel[static_cast<size_t>(k + radius)];
        }
        output[static_cast<size_t>(i)] = sum;
    }
    return output;
}

std::vector<double> CaliperTool::computeGradient(const std::vector<double>& profile) const
{
    std::vector<double> gradient(profile.size(), 0.0);
    if (profile.size() < 2) {
        return gradient;
    }

    gradient.front() = profile[1] - profile[0];
    for (size_t i = 1; i + 1 < profile.size(); ++i) {
        gradient[i] = 0.5 * (profile[i + 1] - profile[i - 1]);
    }
    gradient.back() = profile.back() - profile[profile.size() - 2];
    return gradient;
}

std::vector<EdgePoint> CaliperTool::detectCandidates(const std::vector<double>& gradient,
                                                     const CaliperRegion& region) const
{
    const CaliperParams p = normalizedParams(m_params);
    std::vector<EdgePoint> candidates;
    const int count = static_cast<int>(gradient.size());
    if (count < 3) {
        return candidates;
    }

    const double angleRad = region.angleDeg * pi / 180.0;
    const double dirX = std::cos(angleRad);
    const double dirY = std::sin(angleRad);
    const double step = region.length / static_cast<double>(count - 1);

    for (int i = 1; i + 1 < count; ++i) {
        const double r0 = responseAt(gradient[static_cast<size_t>(i - 1)], p.polarity);
        const double r1 = responseAt(gradient[static_cast<size_t>(i)], p.polarity);
        const double r2 = responseAt(gradient[static_cast<size_t>(i + 1)], p.polarity);
        if (r1 < p.minResponse || r1 < r0 || r1 < r2) {
            continue;
        }

        double delta = 0.0;
        if (p.enableSubpixel) {
            const double denominator = r0 - 2.0 * r1 + r2;
            if (std::abs(denominator) > std::numeric_limits<double>::epsilon()) {
                delta = clampDouble(0.5 * (r0 - r2) / denominator, -1.0, 1.0);
            }
        }

        const double subIndex = static_cast<double>(i) + delta;
        const double position1D = -region.length * 0.5 + subIndex * step;

        EdgePoint edge;
        edge.x = region.centerX + position1D * dirX;
        edge.y = region.centerY + position1D * dirY;
        edge.nx = dirX;
        edge.ny = dirY;
        edge.response = r1;
        edge.position1D = position1D;
        edge.subIndex = subIndex;
        edge.valid = true;
        candidates.push_back(edge);
    }
    return candidates;
}

std::vector<EdgePoint> CaliperTool::selectEdges(const std::vector<EdgePoint>& candidates, bool* selectedByFallback) const
{
    if (selectedByFallback) {
        *selectedByFallback = false;
    }
    if (candidates.empty()) {
        return {};
    }

    const CaliperParams p = normalizedParams(m_params);
    if (p.selection == EdgeSelection::All) {
        return candidates;
    }

    auto selectByStrategy = [&candidates](EdgeSelection selection, double expectedPosition1D) {
        auto selected = candidates.begin();
        switch (selection) {
        case EdgeSelection::First:
            selected = std::min_element(candidates.begin(), candidates.end(), [](const EdgePoint& a, const EdgePoint& b) {
                return a.position1D < b.position1D;
            });
            break;
        case EdgeSelection::Last:
            selected = std::max_element(candidates.begin(), candidates.end(), [](const EdgePoint& a, const EdgePoint& b) {
                return a.position1D < b.position1D;
            });
            break;
        case EdgeSelection::Strongest:
            selected = std::max_element(candidates.begin(), candidates.end(), [](const EdgePoint& a, const EdgePoint& b) {
                return a.response < b.response;
            });
            break;
        case EdgeSelection::NearestToCenter:
            selected = std::min_element(candidates.begin(), candidates.end(), [](const EdgePoint& a, const EdgePoint& b) {
                return std::abs(a.position1D) < std::abs(b.position1D);
            });
            break;
        case EdgeSelection::NearestToExpected:
            selected = std::min_element(candidates.begin(), candidates.end(), [expectedPosition1D](const EdgePoint& a, const EdgePoint& b) {
                return std::abs(a.position1D - expectedPosition1D) < std::abs(b.position1D - expectedPosition1D);
            });
            break;
        case EdgeSelection::All:
            break;
        }
        return selected;
    };

    if (p.selection == EdgeSelection::NearestToExpected) {
        std::vector<EdgePoint> expectedCandidates;
        expectedCandidates.reserve(candidates.size());
        for (const EdgePoint& candidate : candidates) {
            if (std::abs(candidate.position1D - p.expectedPosition1D) <= p.maxPositionDeviation) {
                expectedCandidates.push_back(candidate);
            }
        }
        if (!expectedCandidates.empty()) {
            const auto selected = std::min_element(expectedCandidates.begin(), expectedCandidates.end(), [&p](const EdgePoint& a, const EdgePoint& b) {
                return std::abs(a.position1D - p.expectedPosition1D) < std::abs(b.position1D - p.expectedPosition1D);
            });
            return selected == expectedCandidates.end() ? std::vector<EdgePoint>() : singleEdgeResult(*selected);
        }

        if (!p.allowFallbackSelection) {
            return {};
        }

        EdgeSelection fallback = p.fallbackSelection;
        if (fallback == EdgeSelection::NearestToExpected || fallback == EdgeSelection::All) {
            fallback = EdgeSelection::Strongest;
        }
        const auto selected = selectByStrategy(fallback, p.expectedPosition1D);
        if (selected != candidates.end() && selectedByFallback) {
            *selectedByFallback = true;
        }
        return selected == candidates.end() ? std::vector<EdgePoint>() : singleEdgeResult(*selected);
    }

    auto selected = selectByStrategy(p.selection, p.expectedPosition1D);
    switch (p.selection) {
    case EdgeSelection::First:
    case EdgeSelection::Last:
    case EdgeSelection::Strongest:
    case EdgeSelection::NearestToCenter:
    case EdgeSelection::NearestToExpected:
    case EdgeSelection::All:
        break;
    }

    return selected == candidates.end() ? std::vector<EdgePoint>() : singleEdgeResult(*selected);
}

} // namespace VisionTools
