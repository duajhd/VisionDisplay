#include "shape_match/core/ShapeMatchTypes.h"

#include <algorithm>
#include <cmath>

namespace ShapeMatch {

double radToDeg(double radians)
{
    return radians * 180.0 / kPi;
}

double degToRad(double degrees)
{
    return degrees * kPi / 180.0;
}

double wrapToPi(double radians)
{
    while (radians > kPi) {
        radians -= 2.0 * kPi;
    }
    while (radians < -kPi) {
        radians += 2.0 * kPi;
    }
    return radians;
}

double norm(const cv::Point2d& v)
{
    return std::sqrt(v.x * v.x + v.y * v.y);
}

cv::Point2d normalized(const cv::Point2d& v, const cv::Point2d& fallback)
{
    const double n = norm(v);
    if (n <= 1e-9 || !std::isfinite(n)) {
        return fallback;
    }
    return cv::Point2d(v.x / n, v.y / n);
}

double angleBetweenUnitVectors(const cv::Point2d& a, const cv::Point2d& b)
{
    const cv::Point2d an = normalized(a);
    const cv::Point2d bn = normalized(b);
    const double dot = std::clamp(an.x * bn.x + an.y * bn.y, -1.0, 1.0);
    return std::acos(dot);
}

double MatchPose::thetaDeg() const
{
    return radToDeg(theta);
}

MatchPose MatchPose::fromDeg(double px, double py, double thetaDegValue, double poseScale)
{
    MatchPose pose;
    pose.x = px;
    pose.y = py;
    pose.theta = degToRad(thetaDegValue);
    pose.scale = poseScale;
    return pose;
}

cv::Point2d MatchPose::transformPoint(const cv::Point2d& p) const
{
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    const double sx = p.x * scale;
    const double sy = p.y * scale;
    return cv::Point2d(x + sx * c - sy * s, y + sx * s + sy * c);
}

cv::Point2d MatchPose::rotateVector(const cv::Point2d& v) const
{
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    return cv::Point2d(v.x * c - v.y * s, v.x * s + v.y * c);
}

std::string polarityToString(EdgePolarity polarity)
{
    switch (polarity) {
    case EdgePolarity::Any:
        return "any";
    case EdgePolarity::DarkToBright:
        return "dark_to_bright";
    case EdgePolarity::BrightToDark:
        return "bright_to_dark";
    }
    return "unknown";
}

} // namespace ShapeMatch
