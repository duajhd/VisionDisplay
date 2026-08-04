#include "VisionTools/Matching/ShapeModelBuilder.h"

#include <opencv2/imgproc.hpp>

#include <QColor>
#include <QDir>
#include <QPainter>
#include <QPen>
#include <QStringList>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <numeric>
#include <queue>
#include <unordered_map>
#include <utility>

namespace VisionTools::Matching {

namespace {

constexpr double eps = 1.0e-9;

struct CandidatePoint
{
    int x = 0;
    int y = 0;
    TemplatePoint point;
    bool rejected = false;
};

struct LevelWork
{
    cv::Mat gray;
    cv::Mat blurred;
    cv::Mat ix;
    cv::Mat iy;
    cv::Mat mag;
    std::vector<CandidatePoint> candidates;
    std::vector<EdgeChain> rawChains;
    std::vector<CandidatePoint> rejected;
};

double clampDouble(double value, double low, double high)
{
    return std::max(low, std::min(high, value));
}

double pixelAt(const cv::Mat& image, int x, int y)
{
    x = std::clamp(x, 0, image.cols - 1);
    y = std::clamp(y, 0, image.rows - 1);
    return image.at<float>(y, x);
}

double bilinearAt(const cv::Mat& image, double x, double y)
{
    if (image.empty()) {
        return 0.0;
    }

    x = clampDouble(x, 0.0, static_cast<double>(image.cols - 1));
    y = clampDouble(y, 0.0, static_cast<double>(image.rows - 1));
    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int x1 = std::min(x0 + 1, image.cols - 1);
    const int y1 = std::min(y0 + 1, image.rows - 1);
    const double fx = x - x0;
    const double fy = y - y0;
    const double v00 = image.at<float>(y0, x0);
    const double v10 = image.at<float>(y0, x1);
    const double v01 = image.at<float>(y1, x0);
    const double v11 = image.at<float>(y1, x1);
    return (1.0 - fx) * (1.0 - fy) * v00
        + fx * (1.0 - fy) * v10
        + (1.0 - fx) * fy * v01
        + fx * fy * v11;
}

Eigen::Vector2d normalized(const Eigen::Vector2d& v, const Eigen::Vector2d& fallback)
{
    const double n = v.norm();
    if (n <= eps || !std::isfinite(n)) {
        return fallback;
    }
    return v / n;
}

QImage matToDebugImage(const cv::Mat& gray)
{
    QImage image(gray.cols, gray.rows, QImage::Format_RGB32);
    for (int y = 0; y < gray.rows; ++y) {
        QRgb* row = reinterpret_cast<QRgb*>(image.scanLine(y));
        for (int x = 0; x < gray.cols; ++x) {
            const int v = std::clamp(static_cast<int>(std::round(gray.at<float>(y, x))), 0, 255);
            row[x] = qRgb(v, v, v);
        }
    }
    return image;
}

QImage matToGrayQImage(const cv::Mat& gray)
{
    QImage image(gray.cols, gray.rows, QImage::Format_Grayscale8);
    for (int y = 0; y < gray.rows; ++y) {
        uchar* row = image.scanLine(y);
        for (int x = 0; x < gray.cols; ++x) {
            row[x] = static_cast<uchar>(std::clamp(static_cast<int>(std::round(gray.at<float>(y, x))), 0, 255));
        }
    }
    return image;
}

QColor chainColor(int id)
{
    static const QColor colors[] = {
        QColor(0, 220, 120), QColor(80, 170, 255), QColor(255, 210, 60),
        QColor(255, 90, 120), QColor(190, 120, 255), QColor(80, 230, 230),
        QColor(255, 150, 60), QColor(170, 230, 80)
    };
    return colors[static_cast<size_t>(std::abs(id)) % std::size(colors)];
}

cv::Mat qImageToGrayMat(const QImage& image, QRect roi)
{
    if (image.isNull()) {
        return {};
    }
    const QImage gray = image.format() == QImage::Format_Grayscale8
        ? image
        : image.convertToFormat(QImage::Format_Grayscale8);
    roi = roi.normalized().intersected(QRect(0, 0, gray.width(), gray.height()));
    if (roi.isEmpty()) {
        return {};
    }

    cv::Mat mat(roi.height(), roi.width(), CV_32F);
    for (int y = 0; y < roi.height(); ++y) {
        const uchar* src = gray.constScanLine(roi.y() + y) + roi.x();
        float* dst = mat.ptr<float>(y);
        for (int x = 0; x < roi.width(); ++x) {
            dst[x] = static_cast<float>(src[x]);
        }
    }
    return mat;
}

std::vector<cv::Mat> buildPyramid(const cv::Mat& level0, int requestedLevels)
{
    std::vector<cv::Mat> pyramid;
    requestedLevels = std::max(1, requestedLevels);
    pyramid.push_back(level0.clone());
    for (int i = 1; i < requestedLevels; ++i) {
        const cv::Mat& previous = pyramid.back();
        if (previous.cols < 16 || previous.rows < 16) {
            break;
        }
        cv::Mat next;
        cv::pyrDown(previous, next);
        pyramid.push_back(next);
    }
    return pyramid;
}

void computeGradients(const cv::Mat& gray, const ShapeModelParams& params, LevelWork* work)
{
    const double sigma = std::max(0.0, params.gaussianSigma);
    if (sigma > 0.0) {
        cv::GaussianBlur(gray, work->blurred, cv::Size(), sigma, sigma, cv::BORDER_REPLICATE);
    } else {
        work->blurred = gray.clone();
    }

    cv::Scharr(work->blurred, work->ix, CV_32F, 1, 0, 1.0 / 32.0, 0.0, cv::BORDER_REPLICATE);
    cv::Scharr(work->blurred, work->iy, CV_32F, 0, 1, 1.0 / 32.0, 0.0, cv::BORDER_REPLICATE);
    cv::magnitude(work->ix, work->iy, work->mag);
}

double gradientPercentile(const cv::Mat& magnitude, double minimum, double ratio)
{
    cv::Mat validMask;
    cv::compare(magnitude, minimum, validMask, cv::CMP_GE);
    const int validCount = cv::countNonZero(validMask);
    if (validCount <= 0) return minimum;
    double maximum = minimum;
    cv::minMaxLoc(magnitude, nullptr, &maximum, nullptr, nullptr, validMask);
    if (!(maximum > minimum)) return minimum;

    constexpr int binCount = 512;
    const float range[] = {0.0f, static_cast<float>(maximum) + 1.0f};
    const float* ranges[] = {range};
    const int channels[] = {0};
    const int histogramSize[] = {binCount};
    cv::Mat histogram;
    cv::calcHist(&magnitude, 1, channels, validMask, histogram, 1,
                 histogramSize, ranges, true, false);
    ratio = clampDouble(ratio, 0.0, 1.0);
    const int target = std::max(1, static_cast<int>(std::ceil(ratio * validCount)));
    int cumulative = 0;
    for (int bin = 0; bin < binCount; ++bin) {
        cumulative += static_cast<int>(std::lround(histogram.at<float>(bin)));
        if (cumulative >= target)
            return (static_cast<double>(bin + 1) / binCount) * range[1];
    }
    return maximum;
}

std::vector<CandidatePoint> extractCandidates(const LevelWork& work,
                                              int levelIndex,
                                              double scale,
                                              const Eigen::Vector2d& roiOffset,
                                              const Eigen::Vector2d& originLocal,
                                              const Eigen::Vector2d& originImage,
                                              const ShapeModelParams& params)
{
    const double high = std::max(params.minGradMag,
        gradientPercentile(work.mag, params.minGradMag, params.highThresholdPercentile));
    const double low = std::max(params.minGradMag, high * clampDouble(params.lowThresholdRatio, 0.0, 1.0));

    cv::Mat blurred8;
    work.blurred.convertTo(blurred8, CV_8U);
    cv::Mat edgeMask;
    // work.mag is based on Scharr scaled by 1/32, while Canny's aperture-3
    // Sobel response is approximately eight times larger for an ideal step.
    cv::Canny(blurred8, edgeMask, low * 8.0, high * 8.0, 3, true);
    std::vector<cv::Point> edgePixels;
    cv::findNonZero(edgeMask, edgePixels);

    std::vector<CandidatePoint> candidates;
    candidates.reserve(edgePixels.size());
    for (const cv::Point& pixel : edgePixels) {
        const int x = pixel.x;
        const int y = pixel.y;
        if (x <= 0 || y <= 0 || x + 1 >= work.mag.cols || y + 1 >= work.mag.rows) continue;
        const double gx = work.ix.at<float>(y, x);
        const double gy = work.iy.at<float>(y, x);
        const double m0 = work.mag.at<float>(y, x);
        if (m0 <= eps) continue;
        const Eigen::Vector2d grad(gx, gy);
        const Eigen::Vector2d normal = normalized(grad, Eigen::Vector2d::UnitY());
        const double mm = bilinearAt(work.mag, x - normal.x(), y - normal.y());
        const double mp = bilinearAt(work.mag, x + normal.x(), y + normal.y());
        const double denom = mm - 2.0 * m0 + mp;
        const double offset = std::abs(denom) > eps ? 0.5 * (mm - mp) / denom : 0.0;
        if (!std::isfinite(offset) || std::abs(offset) > params.maxSubpixelOffset) continue;

        CandidatePoint candidate;
        candidate.x = x;
        candidate.y = y;
        candidate.point.level = levelIndex;
        candidate.point.localPos = Eigen::Vector2d(x + offset * normal.x(), y + offset * normal.y());
        candidate.point.imagePos = roiOffset + candidate.point.localPos / scale;
        candidate.point.modelPos = candidate.point.imagePos - originImage;
        candidate.point.pos = candidate.point.modelPos;
        candidate.point.gradient = grad;
        candidate.point.gradMag = m0;
        candidate.point.normal = normal;
        candidate.point.tangent = Eigen::Vector2d(-normal.y(), normal.x());
        candidate.point.response = m0;
        candidate.point.subpixelOffset = offset;
        candidate.point.subpixelScore = std::max(0.0, m0 - 0.5 * (mm + mp));
        candidates.push_back(candidate);
    }
    return candidates;
}

int pointKey(int x, int y, int width)
{
    return y * width + x;
}

double chainLength(const std::vector<TemplatePoint>& points)
{
    double length = 0.0;
    for (size_t i = 1; i < points.size(); ++i) {
        length += (points[i].imagePos - points[i - 1].imagePos).norm();
    }
    return length;
}

std::vector<EdgeChain> buildChains(const std::vector<CandidatePoint>& candidates, int width, int height, int levelIndex)
{
    std::unordered_map<int, int> indexByKey;
    indexByKey.reserve(candidates.size() * 2);
    for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
        indexByKey[pointKey(candidates[static_cast<size_t>(i)].x, candidates[static_cast<size_t>(i)].y, width)] = i;
    }

    std::vector<std::vector<int>> adjacency(candidates.size());
    for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
        const CandidatePoint& p = candidates[static_cast<size_t>(i)];
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) {
                    continue;
                }
                const int nx = p.x + dx;
                const int ny = p.y + dy;
                if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
                    continue;
                }
                const auto it = indexByKey.find(pointKey(nx, ny, width));
                if (it != indexByKey.end()) {
                    adjacency[static_cast<size_t>(i)].push_back(it->second);
                }
            }
        }
    }

    std::vector<char> visited(candidates.size(), 0);
    std::vector<char> inOrdered(candidates.size(), 0);
    std::vector<EdgeChain> chains;
    for (int seed = 0; seed < static_cast<int>(candidates.size()); ++seed) {
        if (visited[static_cast<size_t>(seed)]) {
            continue;
        }

        std::vector<int> component;
        std::queue<int> q;
        q.push(seed);
        visited[static_cast<size_t>(seed)] = 1;
        while (!q.empty()) {
            const int current = q.front();
            q.pop();
            component.push_back(current);
            for (int next : adjacency[static_cast<size_t>(current)]) {
                if (!visited[static_cast<size_t>(next)]) {
                    visited[static_cast<size_t>(next)] = 1;
                    q.push(next);
                }
            }
        }

        int endpoints = 0;
        bool junction = false;
        for (int index : component) {
            const int degree = static_cast<int>(adjacency[static_cast<size_t>(index)].size());
            endpoints += degree <= 1 ? 1 : 0;
            junction = junction || degree > 2;
        }

        int start = component.front();
        if (endpoints > 0) {
            const auto endpoint = std::find_if(component.begin(), component.end(), [&](int index) {
                return adjacency[static_cast<size_t>(index)].size() <= 1;
            });
            if (endpoint != component.end()) {
                start = *endpoint;
            }
        }

        std::vector<int> ordered;
        ordered.reserve(component.size());
        int previous = -1;
        int current = start;
        while (current >= 0) {
            ordered.push_back(current);
            inOrdered[static_cast<size_t>(current)] = 1;
            int best = -1;
            double bestScore = std::numeric_limits<double>::max();
            for (int next : adjacency[static_cast<size_t>(current)]) {
                if (next == previous || inOrdered[static_cast<size_t>(next)]) {
                    continue;
                }
                const Eigen::Vector2d delta = candidates[static_cast<size_t>(next)].point.imagePos
                    - candidates[static_cast<size_t>(current)].point.imagePos;
                const double score = delta.squaredNorm();
                if (score < bestScore) {
                    bestScore = score;
                    best = next;
                }
            }
            previous = current;
            current = best;
        }
        for (int index : component) {
            if (!inOrdered[static_cast<size_t>(index)]) {
                ordered.push_back(index);
                inOrdered[static_cast<size_t>(index)] = 1;
            }
        }
        for (int index : ordered) inOrdered[static_cast<size_t>(index)] = 0;

        EdgeChain chain;
        chain.id = static_cast<int>(chains.size());
        chain.level = levelIndex;
        chain.type = junction ? ChainType::JunctionFragment : (endpoints == 0 ? ChainType::Closed : ChainType::Open);
        chain.points.reserve(ordered.size());
        for (int index : ordered) {
            TemplatePoint point = candidates[static_cast<size_t>(index)].point;
            point.chainId = chain.id;
            point.indexInChain = static_cast<int>(chain.points.size());
            point.isJunction = junction;
            chain.points.push_back(point);
        }
        chain.length = chainLength(chain.points);
        chains.push_back(std::move(chain));
    }
    return chains;
}

void computeChainAttributes(EdgeChain* chain, const LevelWork& work, const ShapeModelParams& params)
{
    if (!chain || chain->points.empty()) {
        return;
    }

    for (int i = 0; i < static_cast<int>(chain->points.size()); ++i) {
        TemplatePoint& point = chain->points[static_cast<size_t>(i)];
        const int prevIndex = std::max(0, i - 2);
        const int nextIndex = std::min(static_cast<int>(chain->points.size()) - 1, i + 2);
        const Eigen::Vector2d delta = chain->points[static_cast<size_t>(nextIndex)].imagePos
            - chain->points[static_cast<size_t>(prevIndex)].imagePos;
        point.tangent = normalized(delta, point.tangent);

        Eigen::Vector2d normal(-point.tangent.y(), point.tangent.x());
        const Eigen::Vector2d gradDir = normalized(point.gradient, normal);
        if (normal.dot(gradDir) < 0.0) {
            normal = -normal;
        }
        point.normal = normalized(normal, gradDir);

        const double orthogonality = std::abs(point.tangent.dot(point.normal));
        if (orthogonality > 0.15) {
            point.isStable = false;
        }

        const double minus = bilinearAt(work.blurred, point.localPos.x() - point.normal.x(), point.localPos.y() - point.normal.y());
        const double plus = bilinearAt(work.blurred, point.localPos.x() + point.normal.x(), point.localPos.y() + point.normal.y());
        point.contrast = plus - minus;
        if (params.usePolarity) {
            point.polarityReliable = std::abs(point.contrast) >= params.minContrast;
            point.polarity = !point.polarityReliable
                ? EdgePolarity::Unknown
                : (point.contrast > 0.0 ? EdgePolarity::DarkToBright : EdgePolarity::BrightToDark);
        }

        if (i > 0 && i + 1 < static_cast<int>(chain->points.size())) {
            const Eigen::Vector2d a = normalized(point.imagePos - chain->points[static_cast<size_t>(i - 1)].imagePos,
                                                 point.tangent);
            const Eigen::Vector2d b = normalized(chain->points[static_cast<size_t>(i + 1)].imagePos - point.imagePos,
                                                 point.tangent);
            point.curvature = std::acos(clampDouble(a.dot(b), -1.0, 1.0));
        }
        point.isCorner = point.curvature >= params.curvatureKeepThreshold;

        const double contrastScore = clampDouble(std::abs(point.contrast) / std::max(params.minContrast, 1.0), 0.0, 2.0);
        const double offsetScore = 1.0 - clampDouble(std::abs(point.subpixelOffset) / std::max(params.maxSubpixelOffset, 0.01), 0.0, 1.0);
        point.stability = 0.45 * clampDouble(point.gradMag / 255.0, 0.0, 1.0)
            + 0.25 * clampDouble(point.subpixelScore / std::max(point.gradMag, 1.0), 0.0, 1.0)
            + 0.20 * clampDouble(contrastScore * 0.5, 0.0, 1.0)
            + 0.10 * offsetScore;
        point.isStable = point.isStable
            && point.gradMag >= params.minGradMag
            && std::abs(point.subpixelOffset) <= params.maxSubpixelOffset
            && (!params.usePolarity || point.polarityReliable);
        point.weight = std::max(0.05, point.stability) * (point.isCorner ? 1.5 : 1.0);
    }
}

std::vector<TemplatePoint> sampleChain(const EdgeChain& chain, int levelIndex, const ShapeModelParams& params)
{
    std::vector<TemplatePoint> selected;
    if (chain.points.empty()) {
        return selected;
    }

    const double spacing = std::max(1.0, params.sampleSpacingLevel0
        * std::pow(std::max(1.0, params.sampleSpacingScaleFactor), levelIndex));
    double distanceSinceKeep = spacing;
    TemplatePoint lastKept;
    bool hasLast = false;

    for (int i = 0; i < static_cast<int>(chain.points.size()); ++i) {
        const TemplatePoint& point = chain.points[static_cast<size_t>(i)];
        if (!point.isStable) {
            continue;
        }

        if (hasLast) {
            distanceSinceKeep += (point.imagePos - lastKept.imagePos).norm();
        }
        const bool keepBySpacing = !hasLast || distanceSinceKeep >= spacing;
        const bool keepCorner = point.isCorner;
        if (keepBySpacing || keepCorner) {
            selected.push_back(point);
            lastKept = point;
            hasLast = true;
            distanceSinceKeep = 0.0;
            continue;
        }

        if (!selected.empty()) {
            TemplatePoint& previous = selected.back();
            const double localDistance = (point.imagePos - previous.imagePos).norm();
            if (localDistance < spacing * 0.5
                && (point.stability > previous.stability || point.gradMag > previous.gradMag * 1.15)) {
                previous = point;
                lastKept = point;
            }
        }
    }

    for (int i = 0; i < static_cast<int>(selected.size()); ++i) {
        selected[static_cast<size_t>(i)].indexInChain = i;
    }
    return selected;
}

void drawDebugImages(TemplateLevel* level, const LevelWork& work, const ShapeModelParams& params)
{
    if (!level || !params.buildDebugImage) {
        return;
    }

    level->allEdgesDebug = matToDebugImage(work.gray);
    level->chainsDebug = matToDebugImage(work.gray);
    level->sampledPointsDebug = matToDebugImage(work.gray);
    level->normalsDebug = matToDebugImage(work.gray);
    level->curvaturePointsDebug = matToDebugImage(work.gray);

    {
        QPainter p(&level->allEdgesDebug);
        p.setPen(QPen(QColor(0, 255, 120), 1));
        for (const CandidatePoint& candidate : work.candidates) {
            p.drawPoint(QPointF(candidate.point.imagePos.x(), candidate.point.imagePos.y()));
        }
        p.setPen(QPen(QColor(255, 50, 50), 1));
        for (const CandidatePoint& candidate : work.rejected) {
            p.drawPoint(QPointF(candidate.point.imagePos.x(), candidate.point.imagePos.y()));
        }
    }

    {
        QPainter p(&level->chainsDebug);
        for (const EdgeChain& chain : work.rawChains) {
            p.setPen(QPen(chainColor(chain.id), 1));
            for (size_t i = 1; i < chain.points.size(); ++i) {
                const auto& a = chain.points[i - 1].imagePos;
                const auto& b = chain.points[i].imagePos;
                p.drawLine(QPointF(a.x(), a.y()), QPointF(b.x(), b.y()));
            }
        }
    }

    {
        QPainter p(&level->sampledPointsDebug);
        p.setPen(QPen(QColor(255, 210, 60), 2));
        for (const TemplatePoint& point : level->points) {
            p.drawEllipse(QPointF(point.imagePos.x(), point.imagePos.y()), 1.5, 1.5);
        }
    }

    {
        QPainter p(&level->normalsDebug);
        p.setPen(QPen(QColor(80, 220, 255), 1));
        int index = 0;
        for (const TemplatePoint& point : level->points) {
            if ((index++ % 5) != 0) {
                continue;
            }
            const QPointF a(point.imagePos.x(), point.imagePos.y());
            const QPointF b(point.imagePos.x() + point.normal.x() * 8.0,
                            point.imagePos.y() + point.normal.y() * 8.0);
            p.drawLine(a, b);
            p.drawEllipse(b, 1.2, 1.2);
        }
    }

    {
        QPainter p(&level->curvaturePointsDebug);
        p.setPen(QPen(QColor(255, 80, 80), 2));
        for (const TemplatePoint& point : level->points) {
            if (point.isCorner) {
                p.drawEllipse(QPointF(point.imagePos.x(), point.imagePos.y()), 2.5, 2.5);
            }
        }
    }
}

void saveDebugImages(const TemplateLevel& level, const ShapeModelParams& params)
{
    if (!params.buildDebugImage) {
        return;
    }

    const QString baseDir = params.debugOutputDir.isEmpty()
        ? QStringLiteral("shape_model_debug")
        : params.debugOutputDir;
    QDir dir;
    if (!dir.exists(baseDir)) {
        dir.mkpath(baseDir);
    }
    if (!dir.cd(baseDir)) {
        return;
    }

    const QString levelDirName = QStringLiteral("level_%1").arg(level.level);
    if (!dir.exists(levelDirName)) {
        dir.mkpath(levelDirName);
    }
    if (!dir.cd(levelDirName)) {
        return;
    }

    level.allEdgesDebug.save(dir.filePath(QStringLiteral("all_edges.png")));
    level.chainsDebug.save(dir.filePath(QStringLiteral("chains.png")));
    level.sampledPointsDebug.save(dir.filePath(QStringLiteral("sampled_points.png")));
    level.normalsDebug.save(dir.filePath(QStringLiteral("normals.png")));
    level.curvaturePointsDebug.save(dir.filePath(QStringLiteral("curvature_points.png")));
}

TemplateLevel buildLevel(const cv::Mat& gray,
                         int levelIndex,
                         double scale,
                         const Eigen::Vector2d& roiOffset,
                         const Eigen::Vector2d& originLocal,
                         const Eigen::Vector2d& originImage,
                         const ShapeModelParams& params)
{
    LevelWork work;
    work.gray = gray;
    computeGradients(gray, params, &work);
    work.candidates = extractCandidates(work, levelIndex, scale, roiOffset, originLocal, originImage, params);
    work.rawChains = buildChains(work.candidates, gray.cols, gray.rows, levelIndex);

    TemplateLevel level;
    level.level = levelIndex;
    level.scale = scale;
    level.width = gray.cols;
    level.height = gray.rows;
    level.origin = originLocal * scale;
    level.image = matToGrayQImage(gray);
    level.allEdgePoints.reserve(work.candidates.size());
    for (const CandidatePoint& candidate : work.candidates) {
        TemplatePoint point = candidate.point;
        point.id = static_cast<int>(level.allEdgePoints.size());
        level.allEdgePoints.push_back(point);
    }

    for (EdgeChain& chain : work.rawChains) {
        computeChainAttributes(&chain, work, params);

        EdgeChain filtered;
        filtered.id = static_cast<int>(level.chains.size());
        filtered.level = levelIndex;
        filtered.type = chain.type;
        for (const TemplatePoint& point : chain.points) {
            if (point.isStable) {
                TemplatePoint copy = point;
                copy.chainId = filtered.id;
                copy.indexInChain = static_cast<int>(filtered.points.size());
                filtered.points.push_back(copy);
            } else {
                CandidatePoint rejected;
                rejected.point = point;
                work.rejected.push_back(rejected);
                level.rejectedPoints.push_back(point);
            }
        }
        filtered.length = chainLength(filtered.points);
        if (static_cast<int>(filtered.points.size()) < params.minChainPoints
            || filtered.length < params.minChainLength) {
            for (const TemplatePoint& point : filtered.points) {
                CandidatePoint rejected;
                rejected.point = point;
                work.rejected.push_back(rejected);
                level.rejectedPoints.push_back(point);
            }
            continue;
        }

        for (const TemplatePoint& point : filtered.points) {
            TemplatePoint stablePoint = point;
            stablePoint.id = static_cast<int>(level.stablePoints.size());
            level.stablePoints.push_back(stablePoint);
        }

        std::vector<TemplatePoint> sampled = sampleChain(filtered, levelIndex, params);
        if (sampled.empty()) {
            continue;
        }
        filtered.points = sampled;
        filtered.length = chainLength(filtered.points);
        for (TemplatePoint& point : filtered.points) {
            point.chainId = filtered.id;
            point.id = static_cast<int>(level.points.size());
            level.points.push_back(point);
        }
        level.chains.push_back(std::move(filtered));
    }

    level.message = QStringLiteral("level=%1 candidates=%2 chains=%3 sampled=%4")
                        .arg(levelIndex)
                        .arg(static_cast<int>(work.candidates.size()))
                        .arg(static_cast<int>(level.chains.size()))
                        .arg(static_cast<int>(level.points.size()));
    drawDebugImages(&level, work, params);
    saveDebugImages(level, params);
    return level;
}

} // namespace

ShapeTemplateModel ShapeModelBuilder::build(const QImage& image,
                                            QRect roi,
                                            const ShapeModelParams& params,
                                            QPointF origin) const
{
    ShapeModelBuildInput input;
    const QRect boundedRoi = roi.normalized().intersected(QRect(0, 0, image.width(), image.height()));
    input.image = qImageToGrayMat(image, boundedRoi);
    input.originalImageSize = cv::Size(image.width(), image.height());
    input.modelRoi = cv::Rect(boundedRoi.x(), boundedRoi.y(), boundedRoi.width(), boundedRoi.height());
    input.roiOffset = Eigen::Vector2d(boundedRoi.x(), boundedRoi.y());
    return build(input, params, origin);
}

ShapeTemplateModel ShapeModelBuilder::build(const ShapeModelBuildInput& input,
                                            const ShapeModelParams& params,
                                            QPointF origin) const
{
    ShapeTemplateModel model;
    model.params = params;
    model.originalImageSize = input.originalImageSize;
    model.modelRoi = input.modelRoi;
    model.roiOffset = input.roiOffset;
    model.sourceRoi = QRect(input.modelRoi.x, input.modelRoi.y, input.modelRoi.width, input.modelRoi.height);
    if (input.image.empty() || input.modelRoi.width <= 0 || input.modelRoi.height <= 0) {
        model.message = QStringLiteral("Shape model build failed: image or ROI is empty");
        return model;
    }

    cv::Mat level0;
    if (input.image.type() == CV_32F) {
        level0 = input.image.clone();
    } else if (input.image.channels() == 1) {
        input.image.convertTo(level0, CV_32F);
    } else {
        cv::Mat gray;
        cv::cvtColor(input.image, gray, cv::COLOR_BGR2GRAY);
        gray.convertTo(level0, CV_32F);
    }

    const bool hasOrigin = std::isfinite(origin.x()) && std::isfinite(origin.y())
        && !(qFuzzyIsNull(origin.x()) && qFuzzyIsNull(origin.y()));
    model.originLocal = hasOrigin
        ? Eigen::Vector2d(origin.x() - input.roiOffset.x(), origin.y() - input.roiOffset.y())
        : Eigen::Vector2d(level0.cols * 0.5, level0.rows * 0.5);
    model.originImage = input.roiOffset + model.originLocal;
    model.origin = model.originLocal;

    const std::vector<cv::Mat> pyramid = buildPyramid(level0, params.pyramidLevels);
    model.levels.reserve(pyramid.size());
    for (int i = 0; i < static_cast<int>(pyramid.size()); ++i) {
        const double scale = 1.0 / static_cast<double>(1 << i);
        model.levels.push_back(buildLevel(pyramid[static_cast<size_t>(i)], i, scale, input.roiOffset, model.originLocal, model.originImage, params));
    }

    QStringList messages;
    for (const TemplateLevel& level : model.levels) {
        messages << level.message;
    }
    model.message = messages.join(QLatin1Char('\n'));
    if (!model.isValid()) {
        model.message.prepend(QStringLiteral("Shape model build produced no stable template points\n"));
    }
    return model;
}

} // namespace VisionTools::Matching
