# AGENTS.md

## Project

This project is VisionDisplayLib, a Qt 6 C++/QML industrial image display component similar to Cognex VisionPro CogDisplay.

Read these documents before coding:

- docs/VisionDisplayLib_Spec.md

## Current status

v0.1 has been completed.

Implemented features:
- VisionDisplayItem can display image
- fitToWindow works
- mouse wheel zoom works
- mouse drag pan works
- image/view coordinate mapping works

## General rules

- Use Qt 6, CMake, C++17 or newer.
- Keep public headers in include/VisionDisplay/.
- Keep source files in src/.
- Keep demo app in examples/DemoApp/.
- Do not implement future versions unless explicitly requested.
- Do not introduce OpenCV, Skia, HALCON, Ceres, or camera SDKs unless explicitly requested.
- v0.2 and v0.3 should still depend only on Qt 6.
- After every task, build the project.
- If build fails, fix the errors.
- If build cannot run because of missing local environment, report the exact command that should be run and the missing dependency.

## Architecture rules

- VisionDisplayItem handles QQuickItem integration and input events.
- CoordinateMapper handles coordinate conversion only.
- GraphicManager handles overlay graphics.
- RoiManager handles ROI objects.
- Overlay graphics and ROI must be separate systems.
- Store all graphic and ROI geometry in image coordinates.
- Stroke width and text size should default to screen-pixel units, not image-coordinate units.
- QML should be thin. Do not put heavy rendering or geometry logic in QML.

## Version discipline

Only implement the requested version.

For each version:
1. Inspect current code.
2. Propose files to add or modify.
3. Implement the version.
4. Build.
5. Summarize changed files and remaining issues.