# Task Board

Date: 2026-05-11

## Current Status

- 当前重点任务：FindCircle / Circle Fitting 偏差调试，以及 Single Caliper candidate 诊断能力完善。
- 当前项目目录没有 `.git`，不能读取 git diff/status；后续任务开始前需要确认版本控制状态。
- 按本次收尾要求，没有继续修改业务代码，没有运行长时间构建，也没有使用 Qt Creator build 目录做额外操作。

## Completed

- 项目入口已收敛到 `main.cpp` + `Main.qml`。
- FindCircle 调试相关能力已存在：
  - synthetic circle / regression circle image。
  - editable circle calipers。
  - FindCircle regression run。
  - robust on/off 对比入口。
  - expected/fitted/error/inlier/outlier 诊断输出。
  - expected circle、search annulus、radial calipers、inlier/outlier、fitted circle 可视化。
- CircleFitter 已具备：
  - response-weighted algebraic fit。
  - RANSAC initial fit。
  - geometric refinement。
  - Huber robust weighting。
  - inlier/outlier 和 RMS/max error 输出。
- Single Caliper 调试已增强：
  - 显示所有 candidates。
  - selected candidate 使用明显不同样式。
  - 诊断框输出 candidates 和 profile peaks。
  - 支持 `projectionWidth`、`projectionCount`、`smoothingSigma`、`minResponse`、`polarity`、`edgeSelection`。
  - 增加 `NearestToExpected`、`expectedPosition1D`、`maxPositionDeviation` 和 fallback selection。

## Not Completed

- 未在本次收尾中重新运行构建。
- 未进行 UI 点击复现和截图验证。
- 未确认 FindCircle 调试入口是否仍完整暴露在当前 `Main.qml` UI。
- 未确认圆拟合偏差时，正确 candidate 是否存在但未被 selected。
- 未确认 `projectionWidth=1` 是否比默认宽度更稳定。
- 未确认固定 `DarkToLight/LightToDark` 是否比 `Any` 更稳定。
- 未清理 build 目录或 Qt Creator 用户文件。

## Next Suggested Work

1. 新任务开始后，先确认是否有 `.git`；如果没有，先建立明确的备份或版本基线。
2. 检查当前 `Main.qml` 是否还保留 FindCircle 操作入口；如果没有，先恢复一个最小 FindCircle 调试面板。
3. 用 Single Caliper Debug 固定同一条圆边，分别测试：
   - 多个 `searchDirectionAngleDeg`
   - `projectionWidth=1` vs 默认宽度
   - `DarkToLight` / `LightToDark` / `Any`
   - `Strongest` vs `NearestToExpected`
4. 在 FindCircle 每个 caliper 中记录 all candidates，而不仅是 selected edge。
5. 如果正确 candidate 存在但 selected 错误，优先改 FindCircle candidate selection；不要先改 CircleFitter。
6. 如果所有 selected edge 都正确但拟合仍偏差，再检查 CircleFitter 的 residual、inlier/outlier、RANSAC threshold 和 Huber 参数。

## Notes

- 本文档是关闭当前 Codex 会话前的交接记录。
- 更详细的 FindCircle 调试记录见 `docs/DebugLog_FindCircle.md`。
