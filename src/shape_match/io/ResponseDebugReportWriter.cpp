#include "shape_match/io/ResponseDebugReportWriter.h"

#include "shape_match/core/ShapeMatchTypes.h"

#include <fstream>
#include <vector>

#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace ShapeMatch {

namespace {

void writeJson(const std::filesystem::path& path, const ResponseDebugReport& report)
{
    nlohmann::json j;
    j["level"] = report.level;
    j["image_width"] = report.imageWidth;
    j["image_height"] = report.imageHeight;
    j["orientation_bin_count"] = report.orientationBinCount;
    j["theta_bin_count"] = report.thetaBinCount;
    j["raw_edge_count"] = report.rawEdgeCount;
    j["used_edge_count"] = report.usedEdgeCount;
    j["response_template_point_count"] = report.responseTemplatePointCount;
    j["total_peaks_before_nms"] = report.totalPeaksBeforeNms;
    j["total_peaks_after_nms"] = report.totalPeaksAfterNms;
    j["build_response_map_ms"] = report.buildResponseMapMs;
    j["build_template_ms"] = report.buildTemplateMs;
    j["evaluate_response_ms"] = report.evaluateResponseMs;
    j["peak_extract_ms"] = report.peakExtractMs;
    j["total_ms"] = report.totalMs;
    j["top_peaks"] = nlohmann::json::array();
    for (const ResponsePeak& peak : report.topPeaks) {
        j["top_peaks"].push_back({
            {"x", peak.pose.x},
            {"y", peak.pose.y},
            {"theta_deg", peak.pose.thetaDeg()},
            {"response_score", peak.responseScore},
            {"theta_bin", peak.thetaBin},
            {"tile_row", peak.tileRow},
            {"tile_col", peak.tileCol}
        });
    }

    std::ofstream out(path);
    if (out.is_open()) {
        out << j.dump(2);
    }
}

void writeCsv(const std::filesystem::path& path, const ResponseDebugReport& report)
{
    std::ofstream out(path);
    if (!out.is_open()) {
        return;
    }
    out << "rank,x,y,theta_deg,response_score,theta_bin,tile_row,tile_col\n";
    for (size_t i = 0; i < report.topPeaks.size(); ++i) {
        const ResponsePeak& peak = report.topPeaks[i];
        out << (i + 1) << ','
            << peak.pose.x << ','
            << peak.pose.y << ','
            << peak.pose.thetaDeg() << ','
            << peak.responseScore << ','
            << peak.thetaBin << ','
            << peak.tileRow << ','
            << peak.tileCol << '\n';
    }
}

void writeSummary(const std::filesystem::path& path, const ResponseDebugReport& report)
{
    std::ofstream out(path);
    if (!out.is_open()) {
        return;
    }
    out << "Orientation Response Debug Summary\n";
    out << "level: " << report.level << '\n';
    out << "imageSize: " << report.imageWidth << "x" << report.imageHeight << '\n';
    out << "rawEdgeCount: " << report.rawEdgeCount << '\n';
    out << "usedEdgeCount: " << report.usedEdgeCount << '\n';
    out << "responseTemplatePointCount: " << report.responseTemplatePointCount << '\n';
    out << "totalPeaksBeforeNms: " << report.totalPeaksBeforeNms << '\n';
    out << "totalPeaksAfterNms: " << report.totalPeaksAfterNms << '\n';
    out << "buildResponseMapMs: " << report.buildResponseMapMs << '\n';
    out << "buildTemplateMs: " << report.buildTemplateMs << '\n';
    out << "evaluateResponseMs: " << report.evaluateResponseMs << '\n';
    out << "peakExtractMs: " << report.peakExtractMs << '\n';
    out << "totalMs: " << report.totalMs << '\n';
    const size_t count = std::min<size_t>(report.topPeaks.size(), 10);
    for (size_t i = 0; i < count; ++i) {
        const ResponsePeak& p = report.topPeaks[i];
        out << "peak " << (i + 1)
            << ": x=" << p.pose.x
            << " y=" << p.pose.y
            << " thetaDeg=" << p.pose.thetaDeg()
            << " score=" << p.responseScore << '\n';
    }
}

bool writeBmpFallback(const std::filesystem::path& path, const cv::Mat& image)
{
    if (image.empty()) {
        return false;
    }
    cv::Mat bgr;
    if (image.channels() == 1) {
        cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
    } else if (image.channels() == 3) {
        bgr = image;
    } else {
        return false;
    }
    if (bgr.type() != CV_8UC3) {
        cv::Mat tmp;
        bgr.convertTo(tmp, CV_8UC3);
        bgr = tmp;
    }

    const int width = bgr.cols;
    const int height = bgr.rows;
    const int rowStride = ((width * 3 + 3) / 4) * 4;
    const int pixelBytes = rowStride * height;
    const int fileSize = 54 + pixelBytes;

    std::ofstream out(path, std::ios::binary);
    if (!out.is_open()) {
        return false;
    }

    unsigned char header[54] = {};
    header[0] = 'B';
    header[1] = 'M';
    header[2] = static_cast<unsigned char>(fileSize);
    header[3] = static_cast<unsigned char>(fileSize >> 8);
    header[4] = static_cast<unsigned char>(fileSize >> 16);
    header[5] = static_cast<unsigned char>(fileSize >> 24);
    header[10] = 54;
    header[14] = 40;
    header[18] = static_cast<unsigned char>(width);
    header[19] = static_cast<unsigned char>(width >> 8);
    header[20] = static_cast<unsigned char>(width >> 16);
    header[21] = static_cast<unsigned char>(width >> 24);
    header[22] = static_cast<unsigned char>(height);
    header[23] = static_cast<unsigned char>(height >> 8);
    header[24] = static_cast<unsigned char>(height >> 16);
    header[25] = static_cast<unsigned char>(height >> 24);
    header[26] = 1;
    header[28] = 24;
    out.write(reinterpret_cast<const char*>(header), sizeof(header));

    std::vector<unsigned char> row(static_cast<size_t>(rowStride), 0);
    for (int y = height - 1; y >= 0; --y) {
        const cv::Vec3b* src = bgr.ptr<cv::Vec3b>(y);
        for (int x = 0; x < width; ++x) {
            row[static_cast<size_t>(x * 3 + 0)] = src[x][0];
            row[static_cast<size_t>(x * 3 + 1)] = src[x][1];
            row[static_cast<size_t>(x * 3 + 2)] = src[x][2];
        }
        out.write(reinterpret_cast<const char*>(row.data()), rowStride);
    }
    return true;
}

void safeWriteImage(const std::filesystem::path& path, const cv::Mat& image)
{
    const std::string extension = path.extension().string();
    if (!extension.empty()) {
        try {
            if (cv::haveImageWriter(extension) && cv::imwrite(path.string(), image)) {
                return;
            }
        } catch (const cv::Exception&) {
        }
    }
    writeBmpFallback(path, image);
}

void writeImages(const std::filesystem::path& dir, const ResponseDebugReport& report)
{
    if (!report.maxResponseXY.empty()) {
        cv::Mat normalized;
        cv::normalize(report.maxResponseXY, normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
        cv::Mat color;
        cv::applyColorMap(normalized, color, cv::COLORMAP_JET);
        safeWriteImage(dir / "latest_response_max_xy.png", color);
    }
    if (!report.bestThetaXY.empty()) {
        cv::Mat best8;
        cv::normalize(report.bestThetaXY, best8, 0, 255, cv::NORM_MINMAX, CV_8U);
        safeWriteImage(dir / "latest_response_best_theta.png", best8);
    }

    cv::Mat overlay;
    if (!report.edgeOverlayBase.empty()) {
        if (report.edgeOverlayBase.channels() == 1) {
            cv::cvtColor(report.edgeOverlayBase, overlay, cv::COLOR_GRAY2BGR);
        } else {
            overlay = report.edgeOverlayBase.clone();
        }
    } else if (!report.maxResponseXY.empty()) {
        overlay = cv::Mat::zeros(report.maxResponseXY.size(), CV_8UC3);
    }
    if (!overlay.empty()) {
        const size_t count = std::min<size_t>(report.topPeaks.size(), 100);
        for (size_t i = 0; i < count; ++i) {
            const ResponsePeak& peak = report.topPeaks[i];
            cv::Point center(static_cast<int>(std::round(peak.pose.x)), static_cast<int>(std::round(peak.pose.y)));
            cv::circle(overlay, center, 4, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
            const cv::Point tip(center.x + static_cast<int>(std::round(12.0 * std::cos(peak.pose.theta))),
                                center.y + static_cast<int>(std::round(12.0 * std::sin(peak.pose.theta))));
            cv::line(overlay, center, tip, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
        }
        safeWriteImage(dir / "latest_response_peaks_overlay.png", overlay);
    }
}

} // namespace

ResponseDebugReportWriter::ResponseDebugReportWriter(std::filesystem::path reportDir)
    : m_reportDir(std::move(reportDir))
{
}

void ResponseDebugReportWriter::write(const ResponseDebugReport& report, const OrientationResponseConfig& config) const
{
    std::filesystem::create_directories(m_reportDir);
    if (config.exportResponseJson) {
        writeJson(m_reportDir / "latest_response_debug.json", report);
    }
    if (config.exportResponseCsv) {
        writeCsv(m_reportDir / "latest_response_peaks.csv", report);
    }
    if (config.exportResponseSummary) {
        writeSummary(m_reportDir / "latest_response_summary.txt", report);
    }
    if (config.exportResponseDebugImages) {
        writeImages(m_reportDir, report);
    }
}

} // namespace ShapeMatch
