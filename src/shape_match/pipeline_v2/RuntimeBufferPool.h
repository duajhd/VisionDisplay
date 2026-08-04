#pragma once

#include <opencv2/core.hpp>

#include <map>
#include <memory>
#include <string>
#include <typeindex>
#include <vector>

namespace ShapeMatch {

class RuntimeBufferPool
{
public:
    void resetForRun();

    cv::Mat& acquireFloatMat(const cv::Size& size, const std::string& name);
    cv::Mat& acquireByteMat(const cv::Size& size, const std::string& name);

    template<typename T>
    std::vector<T>& acquireVector(const std::string& name, size_t reserveSize)
    {
        const VectorKey key{name, std::type_index(typeid(T))};
        auto it = m_vectors.find(key);
        if (it == m_vectors.end()) {
            auto holder = std::make_unique<VectorHolder<T>>();
            holder->values.reserve(reserveSize);
            auto* raw = holder.get();
            m_vectors.emplace(key, std::move(holder));
            ++m_allocatedVectorCount;
            return raw->values;
        }
        auto* holder = static_cast<VectorHolder<T>*>(it->second.get());
        holder->values.clear();
        if (holder->values.capacity() < reserveSize) {
            holder->values.reserve(reserveSize);
        }
        ++m_reusedVectorCount;
        return holder->values;
    }

    void clearAll();

    size_t allocatedMatCount() const;
    size_t reusedMatCount() const;
    size_t allocatedVectorCount() const;
    size_t reusedVectorCount() const;

private:
    struct MatKey {
        std::string name;
        int width = 0;
        int height = 0;
        int type = 0;

        bool operator<(const MatKey& other) const
        {
            return std::tie(name, width, height, type) < std::tie(other.name, other.width, other.height, other.type);
        }
    };

    struct VectorKey {
        std::string name;
        std::type_index type;

        bool operator<(const VectorKey& other) const
        {
            return std::tie(name, type) < std::tie(other.name, other.type);
        }
    };

    struct IVectorHolder {
        virtual ~IVectorHolder() = default;
    };

    template<typename T>
    struct VectorHolder final : IVectorHolder {
        std::vector<T> values;
    };

    cv::Mat& acquireMat(const cv::Size& size, int type, const std::string& name);

    std::map<MatKey, cv::Mat> m_mats;
    std::map<VectorKey, std::unique_ptr<IVectorHolder>> m_vectors;
    size_t m_allocatedMatCount = 0;
    size_t m_reusedMatCount = 0;
    size_t m_allocatedVectorCount = 0;
    size_t m_reusedVectorCount = 0;
};

} // namespace ShapeMatch
