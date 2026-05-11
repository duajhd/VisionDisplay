# VisionDisplay API Overview

VisionDisplay is a Qt 6 / Qt Quick industrial image display component. It is exported as a C++ shared library and a QML module named `VisionDisplay 1.0`.

## Main Type

QML:

```qml
import VisionDisplay 1.0

VisionDisplay {
    id: display
}
```

C++:

```cpp
#include <VisionDisplay/VisionDisplayItem.h>
```

The main public class is `VisionDisplay::VisionDisplayItem`.

## Coordinate System

All image geometry accepted by the public API uses image coordinates:

- Overlay graphics
- ROI geometry
- Inspection result graphics
- Tool result graphics
- Mouse position signals

`imageToView()` and `viewToImage()` are available for explicit conversion when needed. Rendering converts image coordinates to view coordinates internally through `CoordinateMapper`.

## Image Input

Supported frame APIs:

- `setImage(const QImage&)`
- `updateFrame(const QImage&)`
- `updateFrame(const uchar* data, int width, int height, int stride, int pixelFormat)`
- QML-friendly `updateFrame(const QByteArray&, int width, int height, int stride, int pixelFormat)`

`PixelFormat` values:

- `Gray8`
- `RGB888`
- `BGR888`
- `RGBA8888`
- `BGRA8888`

Threading rule: frame update APIs copy the input pixels before returning. If called from a non-owner thread, the item applies the frame through a queued call. Acquisition threads must not access Qt Quick scene graph nodes directly.

## View Control

Properties:

- `zoom`
- `minZoom`
- `maxZoom`
- `showPixelInfo`
- `showCrosshair`
- `interactionMode`
- `selectedRoiId`
- `autoFitOnNewImage`
- `keepViewTransformOnNewImage`

Methods:

- `fitToWindow()`
- `setZoomAt(zoom, viewX, viewY)`
- `zoomIn()`
- `zoomOut()`

All QML properties have NOTIFY signals.

## Overlay Graphics

Basic overlay APIs:

- `addLine(id, x1, y1, x2, y2)`
- `addRect(id, x, y, w, h)`
- `addCircle(id, cx, cy, r)`
- `addText(id, x, y, text)`
- `addCross(id, x, y, size)`
- `addPolyline(id, points)`
- `clearGraphics()`
- `clearGraphicsByLayer(layer)`
- `setLayerVisible(layer, visible)`

Layers:

- `Roi = 10`
- `Result = 20`
- `Measure = 30`
- `Temporary = 40`
- `Debug = 50`

## ROI Editing

ROI APIs:

- `createRectRoi(id, x, y, w, h)`
- `createRotatedRectRoi(id, cx, cy, w, h, angleDeg)`
- `createCircleRoi(id, cx, cy, r)`
- `createLineRoi(id, x1, y1, x2, y2)`
- `deleteSelectedRoi()`
- `clearRois()`
- `exportRoisJsonString()`
- `importRoisJsonString(json)`
- `exportRoisJson(path)`
- `importRoisJson(path)`

Signals:

- `roiCreated(id)`
- `roiSelected(id)`
- `roiChanged(id)`
- `roiDeleted(id)`

ROI objects are managed separately from ordinary overlays and tool graphics.

## Inspection Result Graphics

Industrial inspection result APIs:

- `addDefectBox(id, x, y, w, h, label)`
- `addDefectContour(id, points, label)`
- `addBlobRegion(id, points)`
- `addMatchContour(id, points)`
- `addFittedLine(id, x1, y1, x2, y2)`
- `addFittedCircle(id, cx, cy, r)`
- `setInspectionStatus(ok, text)`
- `addResultGraphics(graphics)`
- `clearResultGraphics()`

`clearResultGraphics()` clears inspection result graphics only. It does not remove ROI.

## Tool Result Graphics

Tool result graphics are display-only records from external algorithms. VisionDisplay does not calculate edges, calipers, fitted lines, fitted circles, distances, or angles.

Group management:

- `clearToolGraphics(toolId)`
- `clearAllToolGraphics()`
- `setToolGraphicsVisible(toolId, visible)`

FindLine display:

- `addFindLineSearchRegion(...)`
- `addLineCaliper(...)`
- `addLineCalipers(toolId, calipers)`
- `addEdgePoint(...)`
- `addEdgePoints(toolId, points)`
- `addFittedLineResult(...)`

FindCircle display:

- `addExpectedCircle(...)`
- `addExpectedArc(...)`
- `addCircleSearchAnnulus(...)`
- `addRadialCaliper(...)`
- `addRadialCalipers(toolId, calipers)`
- `addFittedCircleResult(...)`

Other tool result display:

- `addCaliperResult(...)`
- `addDistanceResult(...)`
- `addAngleResult(...)`
- `addToolStatusText(...)`

Tool graphics are grouped by `toolId`; clearing or hiding one tool does not affect ROI or user-created overlay graphics.

## Export

Persistence and image export APIs:

- `saveImage(path)`
- `saveScreenshot(path, withOverlay)`
- `exportGraphicsJson(path)`
- `exportRoisJson(path)`
- `importRoisJson(path)`
- `exportRoisJsonString()`
- `importRoisJsonString(json)`

`saveImage()` writes the original current frame. `saveScreenshot()` writes the current view state and can include overlay/ROI/tool graphics.

## CMake Usage

After installation:

```cmake
find_package(VisionDisplay CONFIG REQUIRED)
target_link_libraries(MyApp PRIVATE VisionDisplay::VisionDisplay)
```

For QML usage, ensure the installed QML module path is visible to the application, then import `VisionDisplay 1.0`.
