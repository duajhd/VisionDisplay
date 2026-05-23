#pragma once

#include "shape_match/coarse/VotingConfig.h"
#include "shape_match/core/ShapeMatchTypes.h"

#include <opencv2/core.hpp>

#include <unordered_map>
#include <vector>

namespace ShapeMatch {

struct AccKey
{
    int xBin = 0;
    int yBin = 0;
    int thetaBin = 0;

    bool operator==(const AccKey& other) const
    {
        return xBin == other.xBin && yBin == other.yBin && thetaBin == other.thetaBin;
    }
};

struct AccKeyHash
{
    size_t operator()(const AccKey& key) const;
};

struct AccValue
{
    float score = 0.0f;
    int voteCount = 0;
};

struct VotePeak
{
    MatchPose pose;
    float voteScore = 0.0f;
    int voteCount = 0;
    int xBin = 0;
    int yBin = 0;
    int thetaBin = 0;
};

class SparseAccumulator
{
public:
    explicit SparseAccumulator(VotingConfig config);

    void clear();
    void vote(float centerX, float centerY, int thetaBin, float weight);
    std::vector<VotePeak> findPeaks(int topK, cv::Size imageSize) const;

    int nonZeroBinCount() const;
    int totalVoteCount() const;

private:
    bool isSuppressedByKept(const VotePeak& candidate, const std::vector<VotePeak>& kept) const;
    int thetaBinDistance(int a, int b) const;

    VotingConfig m_config;
    std::unordered_map<AccKey, AccValue, AccKeyHash> m_bins;
    int m_totalVoteCount = 0;
};

} // namespace ShapeMatch
