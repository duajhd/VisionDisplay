# Shape Match Evaluation

This module is the reusable evaluation layer for future shape-based matching work.
It intentionally does not implement template extraction, coarse search, nonlinear
refinement, or CUDA acceleration.

## Future Integration Points

- Template modeling should fill `ShapeTemplateModel::points` with normalized
  `TemplatePoint` geometry, normals, tangents, gradient direction, polarity, and
  weights. Synthetic model factories are test fixtures, not industrial modeling.
- Coarse search should pass candidate `MatchPose` values to `CandidateRanker`;
  the ranker scores and sorts candidates without changing how candidates were
  generated.
- Refinement can call `MatchEvaluator` before and after optimization to explain
  whether a refined pose improved coverage, distance, orientation, polarity, and
  inlier ratio.
- VisionDisplay should consume `ShapeMatchOverlayData` through
  `VisionDisplayOverlayAdapter`; UI code should not compute scores.
- A future dataset evaluator can read images and ground truth annotations, then
  aggregate `MultiTargetEvalResult` across samples for Precision / Recall / F1.
- `continuityScore`, `distributionScore`, `stabilityScore`,
  `ambiguityPenalty`, and `occlusionPenalty` are reserved for later stages.
