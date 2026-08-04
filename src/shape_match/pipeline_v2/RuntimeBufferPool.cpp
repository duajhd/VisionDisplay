#include "shape_match/pipeline_v2/RuntimeBufferPool.h"

namespace ShapeMatch {

void RuntimeBufferPool::resetForRun()
{
    m_allocatedMatCount = 0;
    m_reusedMatCount = 0;
    m_allocatedVectorCount = 0;
    m_reusedVectorCount = 0;
}

cv::Mat& RuntimeBufferPool::acquireFloatMat(const cv::Size& size, const std::string& name)
{
    return acquireMat(size, CV_32F, name);
}

cv::Mat& RuntimeBufferPool::acquireByteMat(const cv::Size& size, const std::string& name)
{
    return acquireMat(size, CV_8U, name);
}

cv::Mat& RuntimeBufferPool::acquireMat(const cv::Size& size, int type, const std::string& name)
{
    const MatKey key{name, size.width, size.height, type};
    auto it = m_mats.find(key);
    if (it == m_mats.end()) {
        auto inserted = m_mats.emplace(key, cv::Mat(size, type));
        ++m_allocatedMatCount;
        inserted.first->second.setTo(cv::Scalar());
        return inserted.first->second;
    }
    ++m_reusedMatCount;
    it->second.setTo(cv::Scalar());
    return it->second;
}

void RuntimeBufferPool::clearAll()
{
    m_mats.clear();
    m_vectors.clear();
    resetForRun();
}

size_t RuntimeBufferPool::allocatedMatCount() const { return m_allocatedMatCount; }
size_t RuntimeBufferPool::reusedMatCount() const { return m_reusedMatCount; }
size_t RuntimeBufferPool::allocatedVectorCount() const { return m_allocatedVectorCount; }
size_t RuntimeBufferPool::reusedVectorCount() const { return m_reusedVectorCount; }

} // namespace ShapeMatch
