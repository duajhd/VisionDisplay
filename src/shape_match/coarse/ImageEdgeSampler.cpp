#include "shape_match/coarse/ImageEdgeSampler.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace ShapeMatch {

namespace {

float sampleFloat(const cv::Mat& mat, int x, int y, float fallback = 0.0f)
{
    if (mat.empty() || x < 0 || y < 0 || x >= mat.cols || y >= mat.rows) {
        return fallback;
    }
    return mat.at<float>(y, x);
}

} // namespace

std::vector<ImageVotePoint> ImageEdgeSampler::sample(const EdgeImageData& edgeData, const VotingConfig& config) const
{
    return sample(edgeData, config, nullptr);
}

std::vector<ImageVotePoint> ImageEdgeSampler::sample(const EdgeImageData& edgeData,
                                                     const VotingConfig& config,
                                                     ImageEdgeSamplingStats* stats) const
{
    if (stats) {
        *stats = ImageEdgeSamplingStats();
    }
    if (edgeData.imageSize.width <= 0 || edgeData.imageSize.height <= 0) {
        return {};
    }

    const int rows = std::max(1, config.imageTileRows);
    const int cols = std::max(1, config.imageTileCols);
    std::vector<std::vector<ImageVotePoint>> tiles(static_cast<size_t>(rows * cols));
    auto tileIndex = [&](float x, float y) {
        const int col = std::clamp(static_cast<int>(x * cols / std::max(1, edgeData.imageSize.width)), 0, cols - 1);
        const int row = std::clamp(static_cast<int>(y * rows / std::max(1, edgeData.imageSize.height)), 0, rows - 1);
        return row * cols + col;
    };

    int rawCount = 0;
    auto addPoint = [&](float fx, float fy, cv::Point2d fallbackNormal, float fallbackMag) {
        const int x = std::clamp(static_cast<int>(std::round(fx)), 0, edgeData.imageSize.width - 1);
        const int y = std::clamp(static_cast<int>(std::round(fy)), 0, edgeData.imageSize.height - 1);
        cv::Point2d g(sampleFloat(edgeData.gradX, x, y, static_cast<float>(fallbackNormal.x)),
                      sampleFloat(edgeData.gradY, x, y, static_cast<float>(fallbackNormal.y)));
        if (norm(g) <= 1e-6) {
            g = fallbackNormal;
        }
        if (norm(g) <= 1e-6) {
            return;
        }
        g = normalized(g);
        ImageVotePoint p;
        p.x = fx;
        p.y = fy;
        p.gx = static_cast<float>(g.x);
        p.gy = static_cast<float>(g.y);
        p.gradMag = sampleFloat(edgeData.gradMag, x, y, fallbackMag);
        if (p.gradMag <= 0.0f) {
            p.gradMag = fallbackMag > 0.0f ? fallbackMag : 1.0f;
        }
        p.polarity = 0;
        p.gradBin = angleToBin(std::atan2(g.y, g.x), std::max(1, config.orientationBinCount));
        tiles[static_cast<size_t>(tileIndex(fx, fy))].push_back(p);
        ++rawCount;
    };

    if (!edgeData.edgePoints.empty()) {
        for (size_t i = 0; i < edgeData.edgePoints.size(); ++i) {
            const cv::Point2d& pt = edgeData.edgePoints[i];
            if (!edgeData.isInside(pt)) {
                continue;
            }
            const cv::Point2d normal = i < edgeData.edgeNormals.size() ? edgeData.edgeNormals[i] : cv::Point2d(0.0, 0.0);
            addPoint(static_cast<float>(pt.x), static_cast<float>(pt.y), normal, 1.0f);
        }
    } else if (!edgeData.edgeMap.empty()) {
        for (int y = 0; y < edgeData.edgeMap.rows; ++y) {
            const uchar* row = edgeData.edgeMap.ptr<uchar>(y);
            for (int x = 0; x < edgeData.edgeMap.cols; ++x) {
                if (row[x] == 0) {
                    continue;
                }
                addPoint(static_cast<float>(x), static_cast<float>(y), cv::Point2d(0.0, 0.0), 1.0f);
            }
        }
    }

    std::vector<ImageVotePoint> sampled;
    sampled.reserve(static_cast<size_t>(std::min(config.maxImageVotePoints, rows * cols * config.maxImagePointsPerTile)));
    std::vector<int> keptPerTile;
    keptPerTile.reserve(tiles.size());
    for (std::vector<ImageVotePoint>& tile : tiles) {
        std::sort(tile.begin(), tile.end(), [](const ImageVotePoint& a, const ImageVotePoint& b) {
            return a.gradMag > b.gradMag;
        });
        const int keep = std::min<int>(std::max(0, config.maxImagePointsPerTile), static_cast<int>(tile.size()));
        keptPerTile.push_back(keep);
        for (int i = 0; i < keep; ++i) {
            sampled.push_back(tile[static_cast<size_t>(i)]);
        }
    }

    if (static_cast<int>(sampled.size()) > config.maxImageVotePoints) {
        std::sort(sampled.begin(), sampled.end(), [](const ImageVotePoint& a, const ImageVotePoint& b) {
            return a.gradMag > b.gradMag;
        });
        sampled.resize(static_cast<size_t>(std::max(0, config.maxImageVotePoints)));
    }

    if (stats) {
        stats->rawEdgeCount = rawCount;
        stats->sampledCount = static_cast<int>(sampled.size());
        if (!keptPerTile.empty()) {
            stats->minTileCount = *std::min_element(keptPerTile.begin(), keptPerTile.end());
            stats->maxTileCount = *std::max_element(keptPerTile.begin(), keptPerTile.end());
            stats->avgTileCount = std::accumulate(keptPerTile.begin(), keptPerTile.end(), 0.0)
                / static_cast<double>(keptPerTile.size());
        }
    }
    return sampled;
}

int ImageEdgeSampler::angleToBin(double angleRad, int binCount)
{
    angleRad = wrapToPi(angleRad);
    if (angleRad < 0.0) {
        angleRad += 2.0 * kPi;
    }
    int bin = static_cast<int>(std::floor(angleRad * static_cast<double>(binCount) / (2.0 * kPi)));
    return std::clamp(bin, 0, binCount - 1);
}

} // namespace ShapeMatch
