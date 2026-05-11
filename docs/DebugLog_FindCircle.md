# FindCircle / Circle Fitting Debug Log

Date: 2026-05-11

## Current Problem

- FindCircle / circle fitting 调试过程中，曾出现拟合结果偏差、候选边缘点选择不稳定、以及 Demo 入口混乱导致难以复现的问题。
- 近期还出现过点击 Run Single Caliper 后在 Qt/QML 边界崩溃的问题，崩溃堆栈落在 QML 引擎和 moc 调用附近。该问题和 FindCircle 本身不同，但会影响继续定位圆拟合，因为 FindCircle 依赖 caliper 边缘点。
- 当前项目目录不是 git repository，无法读取 `git diff` 或 `git status` 精确确认本轮变更边界。

## Suspected Causes

- FindCircle 偏差更可能来自 caliper 边缘点输入，而不是 CircleFitter 永久性数学错误：
  - 单个 caliper 的 candidate peak 可能存在多个，`Strongest` 可能选到非真实边。
  - `polarity=Any` 在真实边附近和干扰边同时存在时可能引入错误候选。
  - `projectionWidth` 和 `projectionCount` 的平均采样可能让 profile 峰值漂移。
  - `searchDirectionAngleDeg` 与实际边缘法向不一致时，response 和 position1D 会变化。
- CircleFitter 的鲁棒参数也会影响结果：
  - RANSAC residual / maxResidual / minInlierCount 设置过松或过紧，都可能改变 inlier/outlier。
  - Huber 权重会降低异常点影响，但如果输入点整体偏移，鲁棒拟合不能修正候选点选择错误。

## Changes Already Made

- `VisionToolsLib/include/VisionTools/CircleTypes.h`
  - `CircleFitParams` 包含 RANSAC 和 Huber 相关参数：`enableRansac`、`ransacIterations`、`ransacResidual`、`enableHuber`、`huberDelta`。
  - `CircleFitResult` 包含 input/inlier/outlier 点、RMS、max error、score 和 message。

- `VisionToolsLib/src/CircleFitter.cpp`
  - Circle fitting 使用带 response 权重的代数拟合作为基础。
  - 增加 RANSAC 初始圆估计。
  - 增加几何优化和 Huber robust weight。
  - 输出 inlier/outlier、RMS、maxError、score。

- `VisionToolsLib/src/FindCircleTool.cpp`
  - FindCircle 生成径向 calipers，运行 CaliperTool 收集 edge points，再调用 CircleFitter。
  - 将 fitResult 中的 input/inlier/outlier、RMS、score 等结果回填到 FindCircleResult。

- `IntegratedDemoController.h/.cpp`
  - 增加 FindCircle 调试入口：
    - `generateCircleTestImage`
    - `runFindCircle`
    - `runFindCircleRegression`
    - `runFindCircleRegressionRobustOff`
    - `runFindCircleRegressionCurrentSettings`
  - 增加 synthetic circle / regression circle 图像和 editable circle calipers 初始化。
  - `runFindCircleRegressionInternal` 输出：
    - expected center/radius
    - fitted center/radius
    - center/radius error
    - RMS/max error/score
    - 每个 caliper 的 edge、response、position1D、residual、inlier/outlier
  - 当前还有 `m_circleDiagnostics` 字段保存圆调试文本。

- `ToolResultDisplayAdapter.h/.cpp`
  - `showFindCircle` 在 VisionDisplay 上绘制：
    - expected circle/annulus
    - radial calipers
    - inlier edge points
    - outlier edge points
    - fitted circle
    - status text

- `include/VisionDisplay/VisionDisplayItem.h` / `src/VisionDisplayItem.cpp`
  - 增加或完善 FindCircle 可视化 API：
    - expected circle / search annulus
    - radial calipers
    - fitted circle result
    - edge point marker
  - 增加 editable circle calipers 相关交互。

- `CMakeLists.txt`, `main.cpp`, `Main.qml`
  - 项目已被整理为单一入口：`main.cpp` + `Main.qml`。
  - 旧的多 demo 入口不应再作为后续验证入口。

- Single Caliper 调试也已增强，目的是辅助 FindCircle 定位输入点问题：
  - 显示所有 candidate edge points。
  - selected candidate 使用不同样式。
  - 输出 profile peaks 和完整候选列表。
  - 增加 `NearestToExpected`、`expectedPosition1D`、`maxPositionDeviation`、fallback selection。

## Verified Results

- 最近一次构建曾通过：
  - `cmake --build build\Desktop_Qt_6_8_3_MSVC2022_64bit-Debug --config Debug`
  - 输出链接成功：`appVisionDisplay.exe`
- 本次收尾按要求没有重新运行构建。
- 本次无法验证 git diff，因为当前目录没有 `.git`。

## Remaining Issues

- 需要在 UI 中实际对同一张图、同一圆边缘重复运行，确认偏差来自：
  - Caliper candidate 选错；
  - profile/采样/极性导致真实边附近没有 candidate；
  - CircleFitter inlier/outlier 策略误分类。
- `Main.qml` 当前以 Single Caliper Debug 为主屏；FindCircle 调试入口和统一 UI 是否仍完整可用，需要下次会话检查。
- 当前 `runFindCircleRegressionInternal` 的诊断文本主要输出到 `qDebug()` 和 status，没有确认是否在主 UI 中完整显示。
- 没有 git 基线，无法保证哪些变更属于本轮，建议下一次先初始化或恢复版本控制上下文。

## Suggested Next Steps

1. 不先改 CircleFitter，先用 Single Caliper Debug 在圆的若干角度上比较 candidate：
   - 固定 center/radius/searchLength。
   - 改 `searchDirectionAngleDeg`。
   - 记录 candidates 数量、response、position1D。

2. 对 FindCircle 的每个 caliper 输出增加 UI 展示或导出：
   - caliper index
   - selected edge
   - all candidates
   - selected 是否接近 expected radius
   - inlier/outlier

3. 用 `projectionWidth=1` 和默认宽度分别跑同一条边：
   - 如果 `projectionWidth=1` 更稳定，说明宽投影平均引入了邻近结构。
   - 如果两者都不稳定，优先检查 polarity/search direction/profile peak。

4. 分别测试 `DarkToLight`、`LightToDark`、`Any`：
   - 如果固定 polarity 更稳定，避免在 FindCircle 中默认使用 `Any`。

5. 当确认真实边附近存在正确 candidate 但 selected 错误时：
   - 优先使用 `NearestToExpected` 或按 expected radius 的策略选择 candidate。
   - 再考虑是否需要给 FindCircle caliper 增加 expected position 参数。

6. 只有在确认输入 edge points 正确但拟合仍偏差时，再继续调 CircleFitter：
   - 调整 RANSAC threshold。
   - 检查 inlier residual 分布。
   - 比较 robust on/off 的差异。
