# Shape Match 算法原理与执行链路

本文档用于从底层分析当前 shape-based matching 的耗时来源。重点描述当前项目中的实际实现链路、关键数据结构、耗时统计入口和已知瓶颈，而不是通用算法介绍。

## 1. 当前算法定位

当前 Shape Match 是一个以轮廓点为核心的粗定位匹配系统，目标是对标 VisionPro PatMax / HALCON shape_based_matching 的粗定位能力。

当前已实现内容：

- 模板轮廓点建模；
- 图像边缘点提取；
- 多层金字塔；
- RT-table / Orientation Voting 候选生成；
- Distance Field 快速打分；
- Greedy Upper-Bound Pruning；
- Candidate Budget / Beam / Spatial Diversity；
- ROI-local Distance Field；
- Final Ranker / MatchEvaluator 高性能化；
- VisionPro GT 读取；
- 漏检阶段追踪报告；
- 执行时间 profile 报告；
- Debug overlay 绘制。

当前尚未做的内容：

- 最终精定位；
- CUDA；
- SIMD；
- 完整 PatMax 级别的形变/非线性鲁棒优化。

## 2. 总体执行链路

真实图片匹配入口主要在：

- `ShapeModelDebugViewModel::runCoarseMatchAsync()`
- `CoarseShapeMatcher::match()`

整体链路如下：

```text
输入图像 / 模板 ROI / VisionPro GT
  ↓
EdgeImageData 提取图像边缘、梯度、edgePoints
  ↓
ShapeTemplateModel 构建模板轮廓点
  ↓
TemplatePyramid 构建模板金字塔
  ↓
ImagePyramid 构建图像金字塔
  ↓
高层 Orientation Voting 生成初始候选
  ↓
逐层 local refine / beam propagation
  ↓
level0 ROI distance field 构建与局部打分
  ↓
final coarse candidates 选择
  ↓
CandidateRanker / MatchEvaluator 精排
  ↓
MultiTargetEvaluator 对 GT 评估 recall
  ↓
Reports / CSV / TXT / Overlay 输出
```

## 3. 输入与模板建模

### 3.1 图像边缘数据

核心结构：

- `EdgeImageData`
- 文件：`src/shape_match/core/EdgeImageData.*`

主要字段：

- `imageSize`
- `edgeMap`
- `gradX`
- `gradY`
- `gradMag`
- `orientationMap`
- `distanceMap`
- `nearestEdgeX`
- `nearestEdgeY`
- `edgePoints`
- `edgeNormals`

当前边缘提取入口在 demo/debug 层：

- `ShapeModelDebugViewModel::edgeDataFromImage()`

这一步会从 `QImage` 转灰度图，计算 Sobel 梯度，再按梯度幅值阈值生成 edgeMap / edgePoints。

耗时关注点：

- 大图 Sobel；
- 梯度幅值统计；
- edgeMap 生成；
- edgePoints 数量；
- 后续是否对所有 level 构建 distance field。

### 3.2 模板模型

核心结构：

- `ShapeTemplateModel`
- `TemplatePoint`
- 文件：`src/shape_match/core/ShapeTemplateModel.h`

模板点保存：

- 模板局部坐标；
- normal / orientation；
- polarity；
- weight；
- 原点 `origin`。

VisionPro GT 模式下，模板原点来自 GT JSON：

- `training.origin`
- `pose_convention.origin = template_origin`

这保证本算法的 `MatchPose(x,y,theta)` 与 CogPMAlignTool 导出的位姿语义一致。

## 4. 图像金字塔与 Distance Field

### 4.1 ImagePyramid

核心模块：

- `ImagePyramid`
- `DistanceFieldBuilder`
- `RoiDistanceFieldBuilder`

文件：

- `src/shape_match/coarse/ImagePyramid.*`
- `src/shape_match/coarse/DistanceFieldBuilder.*`
- `src/shape_match/coarse/RoiDistanceField.*`
- `src/shape_match/coarse/RoiDistanceFieldBuilder.*`

当前策略：

```text
level3 / level2:
  允许构建全图 distance field

level1:
  默认仍构建全图 distance field

level0:
  大图模式默认跳过全图 distance field
  改为只对 candidate regions 构建 ROI-local distance field
```

关键配置：

- `CoarseMatchConfig::RoiDistanceFieldConfig`
- `buildFullDistanceFieldForLevel0 = false`
- `buildFullDistanceFieldForLevel1 = true`
- `buildFullDistanceFieldForCoarseLevels = true`
- `maxTotalRoiPixelsForDistanceField`
- `roiPaddingPxLevel0`

当前 20MP 大图 profile 中，level0 全图 distance field 已跳过：

```text
skippedFullLevel0DistanceField: true
fullDistanceFieldBuildTimeMsLevel0: 0
```

但是 ROI distance field 仍然可能很贵：

```text
roiDistanceFieldCount: 22
roiDistanceFieldTotalPixels: 约 11182092
roiDistanceFieldBuildTimeMs: 约 3.5s - 4.4s
```

这说明 ROI 总像素仍然很大，后续优化要重点减少 ROI 总面积、合并策略和 ROI 内 distance transform 成本。

## 5. 高层候选生成：Orientation Voting

核心模块：

- `OrientationVotingCandidateGenerator`
- `RTTable`
- `SparseAccumulator`
- `ImageEdgeSampler`

文件：

- `src/shape_match/coarse/OrientationVotingCandidateGenerator.*`
- `src/shape_match/coarse/RTTable.*`
- `src/shape_match/coarse/SparseAccumulator.*`
- `src/shape_match/coarse/ImageEdgeSampler.*`

原理：

1. 模板点相对于模板原点建立 RT-table；
2. 图像边缘点按 orientation bin 查询 RT-table；
3. 每个图像边缘点对可能的模板原点投票；
4. `SparseAccumulator` 找局部峰值；
5. voting peak 经过 `FastPoseScorer` 验证；
6. 形成最高层初始候选。

主要阶段报告：

- `voting_raw_peaks`
- `voting_verified`
- `after_fast_scoring`
- `after_upper_bound_pruning`
- `after_nms`
- `after_spatial_diversity`
- `level_output_beam`

耗时通常不是当前主瓶颈，但影响召回。若漏检发生在早期，应看：

- ROI 内 raw edge points；
- sampled vote points；
- voting peak near target；
- `latest_missed_target_stage_trace.*`。

## 6. Coarse Local Refine / Beam Propagation

核心模块：

- `CoarsePoseGenerator`
- `FastPoseScorer`
- `ParallelCandidateScorer`
- `CandidateBudgetPolicy`
- `SpatialDiversityTopKBuffer`

文件：

- `src/shape_match/coarse/CoarsePoseGenerator.*`
- `src/shape_match/coarse/FastPoseScorer.*`
- `src/shape_match/coarse/ParallelCandidateScorer.*`
- `src/shape_match/coarse/CandidateBudgetPolicy.*`
- `src/shape_match/coarse/SpatialDiversityTopKBuffer.*`

逐层流程：

```text
level_initial_candidates
  ↓
parent_before_selection
  ↓
parent_after_selection
  ↓
local_children_generated
  ↓
after_candidate_budget
  ↓
after_fast_scoring
  ↓
after_upper_bound_pruning
  ↓
after_nms
  ↓
after_spatial_diversity
  ↓
level_output_beam
```

### 6.1 Parent Selection

进入某层 refine 前，会对上一层 beam 结果做父候选筛选。

level0 当前启用 Budget V2：

- `maxLevel0ParentCandidates = 180`
- `maxLevel0ChildrenPerParent = 240`
- `maxLevel0RawChildren = 120000`
- `maxLevel0ScoredChildren = 24000`

最新 profile：

```text
level0ParentBefore: 500
level0ParentAfter: 180
level0RawChildren: 120000
level0ScoredChildren: 24000
```

这说明 level0 仍然生成和打分了大量候选，是当前大图性能瓶颈之一。

### 6.2 Children Generation

每个 parent 在局部平移/角度窗口内生成 child pose。

成本约为：

```text
parent_count × translation_steps × angle_steps
```

后续优化方向：

- 根据 parent score 动态缩小窗口；
- 高置信 parent 用小步长小半径；
- 低置信但空间独特 parent 少量保留；
- per-region 限制 children；
- 先做 cheap duplicate filter；
- 对 level0 不做过大窗口。

## 7. FastPoseScorer

核心模块：

- `FastPoseScorer`

作用：

- 对 coarse/local candidates 快速评分；
- 使用模板采样点；
- 使用 distance field 或 ROI distance field 查询最近边缘；
- 计算 coverage / orientation / polarity 近似分；
- 支持 greedy upper-bound pruning。

关键配置：

- `enableDistanceFieldScoring`
- `enableNearestEdgeField`
- `enableGreedyUpperBoundPruning`
- `upperBoundMargin`
- `minEvaluatedPointsBeforePruning`
- `maxTemplatePointsPerLevel`

查询链路：

```text
FastPoseScorer
  ↓
EdgeQueryContext
  ↓
ROI distance field 优先
  ↓
full distance field fallback
  ↓
local window fallback
```

禁止回退到全图 `edgePoints` 线性扫描。

最新报告里：

```text
roiDistanceFieldQueryCount > 0
fullDistanceFieldQueryCount = 0
linearScanFallbackCount = 0
```

说明当前 level0 scoring 已走 ROI distance field。

## 8. ROI Distance Field

核心结构：

- `RoiDistanceField`
- `RoiDistanceFieldSet`
- `EdgeQueryContext`

文件：

- `src/shape_match/coarse/RoiDistanceField.*`

每个 ROI field 保存：

- `roi`：当前 level 坐标；
- `roiLevel0`：对应 level0 坐标；
- `localEdgeData`：ROI-local 的 edge / gradient / distance map；
- `buildTimeMs`；
- `pixelCount`。

查询过程：

```text
global point
  ↓
findContaining(level, point)
  ↓
global -> ROI-local
  ↓
localEdgeData.findNearestEdgeFast()
  ↓
matched point ROI-local -> global
```

耗时关注点：

- ROI 数量；
- ROI 总像素；
- ROI 是否过度重叠；
- ROI merge 后是否仍然过大；
- ROI 内 distance transform；
- 查询 missingFieldCount 是否过高。

当前一个重要现象：

```text
roiDistanceFieldTotalPixels 约 11.18MP
```

对于 20MP 原图，这已经超过一半图像面积，ROI 化收益有限。后续需要进一步收缩 active regions。

## 9. Final Coarse Candidates 选择

当前 `CoarseShapeMatcher` 在 level0 beam 后，会选择进入 final ranker 的候选。

文件：

- `src/shape_match/coarse/CoarseShapeMatcher.cpp`

当前策略：

- 按 score 排序；
- 过滤近重复 pose；
- 保留空间多样性；
- 保证 final ranker 输入约为 `finalTopK`，当前 debug 配置为 160。

这一步影响多目标召回。如果多个候选集中在少数目标周围，其他目标可能无法进入 final ranker。

相关报告字段：

- `final_coarse_poses`
- `final_ranked_candidates`
- `candidateCountBeforeNms`
- `candidateCountAfterNms`

## 10. Final Ranker / MatchEvaluator

核心模块：

- `CandidateRanker`
- `FinalCandidateNms`
- `MatchEvaluator`
- `PoseErrorEvaluator`
- `MultiTargetEvaluator`

文件：

- `src/shape_match/evaluation/CandidateRanker.*`
- `src/shape_match/evaluation/FinalCandidateNms.*`
- `src/shape_match/evaluation/MatchEvaluator.*`
- `src/shape_match/evaluation/PoseErrorEvaluator.*`
- `src/shape_match/evaluation/MultiTargetEvaluator.*`

### 10.1 CandidateRanker

流程：

```text
final coarse candidates
  ↓
FinalCandidateNms
  ↓
并行 MatchEvaluator score-only
  ↓
排序
  ↓
topK point evaluations 可选重算
  ↓
输出 finalRankedCandidates
```

当前配置：

- `enableDistanceFieldEvaluation = true`
- `enableNearestEdgeFieldEvaluation = true`
- `enableParallelFinalRanking = true`
- `enablePointEvaluations = false`
- `pointEvaluationTopK = 3`
- `maxFinalRankerInputCandidates = 160`
- `targetFinalRankerInputCandidates = 160`

### 10.2 MatchEvaluator

对每个候选：

1. 将模板点按 pose 变换到图像坐标；
2. 查询最近边缘；
3. 计算距离误差；
4. 计算方向误差；
5. 计算 polarity；
6. 汇总 coverage / distance / orientation / polarity / inlier；
7. 输出 `MatchScore`。

重要优化：

过去 `EdgeImageData::findNearestEdge()` 会线性扫描全图 `edgePoints`，20MP 大图上造成灾难耗时。

现在默认使用：

```text
ROI distance field / full distance field O(1) 查询
```

报告中应确认：

```text
linearScanFallbackCount = 0
```

### 10.3 accepted 与 poseOk 的区别

`accepted` 是匹配质量判断：

- coverage；
- inlier；
- median error；
- p90 error；
- valid points。

`poseOk` 是 GT 评估判断：

- `xyOkThresholdPx`
- `angleOkThresholdDeg`
- `scaleOkThreshold`

当前为了粗定位，已调整为：

```text
xyOkThresholdPx = 5.0
angleOkThresholdDeg = 5.0
```

所以某些候选可能：

```text
poseOk = true
accepted = false
```

这表示粗定位位置是对的，但轮廓质量分偏低。Debug overlay 当前允许在有 GT 时补画这些候选，用于分析。

## 11. 报告文件与读法

报告目录：

```text
build/Desktop_Qt_6_8_3_MSVC2022_64bit-Debug/data/shape_match/reports
```

常用报告：

```text
latest_large_image_summary.txt
latest_final_ranker_profile.json
latest_roi_distance_field_profile.json
latest_roi_distance_field_profile.csv
latest_level0_budget_v2.csv
latest_missed_target_stage_trace.json
latest_missed_target_stage_trace.csv
latest_missed_target_stage_trace.txt
latest_coarse_candidates.csv
```

### 11.1 latest_large_image_summary.txt

用于看总耗时和大阶段瓶颈。

重点字段：

- `totalTimeMs`
- `fullDistanceFieldBuildTimeMsTotal`
- `roiDistanceFieldBuildTimeMs`
- `localRefineTotalMs`
- `fastScoreTotalMs`
- `finalRankerMs`
- `recall`
- `roiDistanceFieldQueryCount`
- `linearScanFallbackCount`
- `level0RawChildren`
- `level0ScoredChildren`

### 11.2 latest_final_ranker_profile.json

用于看 final ranker。

重点字段：

- `candidateCountBeforeNms`
- `candidateCountAfterNms`
- `candidateCountActuallyEvaluated`
- `evaluationMode`
- `numThreads`
- `scoreOnlyTimeMs`
- `pointEvalTimeMs`
- `avgTimePerCandidateMs`
- `avgTimePerTemplatePointUs`
- `linearScanFallbackCount`

### 11.3 latest_missed_target_stage_trace.*

用于看目标在哪个阶段丢失。

重点看每个 target：

- `Final hit`
- `Oracle fast score`
- `First missing stage`
- `Likely reason`
- `final_coarse_poses`
- `final_ranked_candidates`

如果某目标：

```text
level_output_beam 有近邻
final_coarse_poses 没近邻
```

说明 final candidate selection 有问题。

如果：

```text
final_coarse_poses 有近邻
final_ranked_candidates 没近邻
```

说明 final ranker NMS / 输入截断 / 精排排序有问题。

如果：

```text
final_ranked_candidates 有近邻
Final hit=false
```

说明姿态误差超过 GT 评估阈值，或 angle/scale 超阈值。

## 12. 当前 20MP 大图耗时拆解

最近一次典型 profile：

```text
totalTimeMs: 约 20.2s
fullDistanceFieldBuildTimeMsTotal: 约 2.0s
roiDistanceFieldBuildTimeMs: 约 4.0s
localRefineTotalMs: 约 10.3s
fastScoreTotalMs: 约 4.2s
finalRankerMs: 约 5.8s

level0ParentBefore: 500
level0ParentAfter: 180
level0RawChildren: 120000
level0ScoredChildren: 24000

candidateCountActuallyEvaluated: 160
linearScanFallbackCount: 0
recall: 0.958
```

注意这些阶段时间并非全部互斥，有些统计口径包含子阶段，因此不能简单相加等于 total。

当前主要瓶颈优先级：

1. `localRefineTotalMs`
2. `roiDistanceFieldBuildTimeMs`
3. `finalRankerMs`
4. `fastScoreTotalMs`
5. level1 full distance field build

## 13. 从底层优化的建议顺序

### 13.1 先固定质量基线

先确定一套验收标准：

```text
GT recall >= 0.95
linearScanFallbackCount = 0
obj_017 单独跟踪
```

不要为了速度降低召回。

### 13.2 优化 level0 local refine

当前：

```text
level0RawChildren = 120000
level0ScoredChildren = 24000
```

优先目标：

```text
level0ScoredChildren <= 12000
```

可做方向：

- parent score 分层；
- 空间唯一 parent 保护；
- 高置信小窗口；
- 低置信少 children；
- per-region children cap；
- child pose hash 去重提前；
- angle search 自适应；
- 对同一目标附近重复 parent 做合并。

### 13.3 优化 ROI distance field

当前 ROI 总像素约 11MP，过大。

优化方向：

- 减少 candidate region 数；
- 改进 ROI merge，避免过大合并块；
- 基于模板 bbox 与不确定性生成更紧 ROI；
- 对没有进入 level0 parent 的 region 不建 field；
- ROI field lazy build：只有实际查询到 region 才构建；
- cache 重用相邻运行中的 ROI field；
- 并行构建 ROI distance field。

### 13.4 优化 FastPoseScorer 内层

关注：

- 每候选模板点数；
- 点排序；
- upper bound pruning 命中率；
- ROI distance field 查询开销；
- branch 数；
- Mat 随机访问；
- orientation / polarity 是否可延迟。

可做：

- 将模板点 SoA 进一步用于内层连续访问；
- 预计算 sin/cos；
- 减少 `cv::Mat::at`；
- 使用 raw pointer；
- 按候选批处理；
- 更早 upper-bound exit。

### 13.5 优化 final ranker

当前 final ranker 评估 160 个候选。

可以做两档：

- Debug/GT 模式：160，保召回；
- Production 模式：50-80，依赖更好的 final candidate diversity。

优化方向：

- score-only 完全禁用 point eval；
- overlay 不触发额外重算；
- final candidate NMS 更精确；
- 先用 cheap score 预筛；
- final ranker ROI query missingFieldCount 降低；
- 多线程粒度优化。

## 14. 当前剩余质量问题

最新报告中只有 `obj_017` 真漏检：

```text
obj_017:
  final_ranked nearest_dxy 约 594px
```

这不是阈值问题，而是候选传播或 local refine 链路中目标已经偏离。

应使用：

```text
latest_missed_target_stage_trace.txt
```

逐层看：

- voting 是否有 near peak；
- level2 是否还 near；
- level1 是否丢；
- level0 parent 是否偏移；
- local children 是否没覆盖；
- budget / NMS / spatial diversity 是否压掉。

## 15. 推荐下一步工作

建议下一阶段不要再先动 final ranker，而是做：

1. 为 `obj_017` 做单目标 stage trace 定位；
2. 输出每层 parent/child 的局部窗口覆盖范围；
3. 统计每个 GT 对应的 nearest parent 是否进入 level0 parent_after_selection；
4. 对 level0 parent selection 加 GT 无关的空间保护策略；
5. 将 ROI field 总像素从 11MP 降到 4-6MP；
6. 将 level0 scored children 从 24000 降到 12000；
7. 保持 `recall >= 0.95` 后再继续压 final ranker 输入数量。
