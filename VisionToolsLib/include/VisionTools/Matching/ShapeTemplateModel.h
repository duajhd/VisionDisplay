#pragma once

#include "VisionTools/VisionTools_global.h"

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <QImage>
#include <QRect>
#include <QString>

#include <vector>

namespace VisionTools::Matching {

enum class EdgePolarity {
    Unknown = 0,
    DarkToBright,
    BrightToDark
};

enum class ChainType {
    Open = 0,
    Closed,
    JunctionFragment
};

struct VISIONTOOLS_API ShapeModelParams
{
    int pyramidLevels = 3;
    double gaussianSigma = 1.0;
    double lowThresholdRatio = 0.35;
    double highThresholdPercentile = 0.85;
    double minGradMag = 8.0;
    double minContrast = 8.0;
    double maxSubpixelOffset = 0.75;
    int minChainPoints = 8;
    double minChainLength = 12.0;
    double sampleSpacingLevel0 = 4.0;
    double sampleSpacingScaleFactor = 1.25;
    double curvatureKeepThreshold = 0.18;
    bool usePolarity = true;
    bool buildDebugImage = false;
    QString debugOutputDir;
};

struct VISIONTOOLS_API ShapeModelBuildInput
{
    cv::Mat image;
    cv::Size originalImageSize;
    cv::Rect modelRoi;
    Eigen::Vector2d roiOffset = Eigen::Vector2d::Zero();
};

struct VISIONTOOLS_API TemplatePoint
{
    int id = -1;
    int level = 0;
    int chainId = -1;
    int indexInChain = -1;
    Eigen::Vector2d pos = Eigen::Vector2d::Zero();
    Eigen::Vector2d localPos = Eigen::Vector2d::Zero();
    Eigen::Vector2d modelPos = Eigen::Vector2d::Zero();
    Eigen::Vector2d imagePos = Eigen::Vector2d::Zero();
    Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
    double gradMag = 0.0;
    Eigen::Vector2d tangent = Eigen::Vector2d::UnitX();
    Eigen::Vector2d normal = Eigen::Vector2d::UnitY();
    EdgePolarity polarity = EdgePolarity::Unknown;
    bool polarityReliable = false;
    double curvature = 0.0;
    double response = 0.0;
    double contrast = 0.0;
    double subpixelOffset = 0.0;
    double subpixelScore = 0.0;
    double stability = 0.0;
    double weight = 1.0;
    bool isStable = true;
    bool isCorner = false;
    bool isJunction = false;
};

struct VISIONTOOLS_API EdgeChain
{
    int id = -1;
    int level = 0;
    ChainType type = ChainType::Open;
    std::vector<TemplatePoint> points;
    double length = 0.0;
};

struct VISIONTOOLS_API TemplateLevel
{
    int level = 0;
    double scale = 1.0;
    int width = 0;
    int height = 0;
    Eigen::Vector2d origin = Eigen::Vector2d::Zero();
    QImage image;
    std::vector<EdgeChain> chains;
    std::vector<TemplatePoint> points;
    std::vector<TemplatePoint> allEdgePoints;
    std::vector<TemplatePoint> stablePoints;
    std::vector<TemplatePoint> rejectedPoints;
    QImage allEdgesDebug;
    QImage chainsDebug;
    QImage sampledPointsDebug;
    QImage normalsDebug;
    QImage curvaturePointsDebug;
    QString message;
};

struct VISIONTOOLS_API ShapeTemplateModel
{
    ShapeModelParams params;
    QRect sourceRoi;
    cv::Size originalImageSize;
    cv::Rect modelRoi;
    Eigen::Vector2d roiOffset = Eigen::Vector2d::Zero();
    Eigen::Vector2d originLocal = Eigen::Vector2d::Zero();
    Eigen::Vector2d originImage = Eigen::Vector2d::Zero();
    Eigen::Vector2d origin = Eigen::Vector2d::Zero();
    std::vector<TemplateLevel> levels;
    QString message;

    bool isValid() const;
    int totalPointCount() const;
};

} // namespace VisionTools::Matching
