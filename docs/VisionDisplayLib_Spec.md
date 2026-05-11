# VisionDisplayLib 项目规格说明

> 目标：使用 Qt 6 + C++ + QML 实现一个类似 Cognex VisionPro `CogDisplay` 的工业图像显示组件，并封装成可复用 DLL / QML 插件，供其他 Qt/QML 工业视觉项目调用。

---

## 1. 项目定位

本项目不是普通图片查看器，而是工业机器视觉软件中的核心显示控件。

目标组件应支持：

- 高性能图像显示
- 图像缩放、平移、适应窗口
- 图像坐标与控件坐标互相转换
- 鼠标交互
- Overlay 图元叠加
- ROI 绘制与编辑
- 检测结果显示
- 图像测量辅助工具
- 实时相机图像刷新
- 截图与数据导出
- DLL 动态库封装
- QML 类型注册，供 QML 直接使用

最终使用方式示例：

```qml
import VisionDisplay 1.0

VisionDisplay {
    id: display
    anchors.fill: parent

    onMouseImagePositionChanged: function(x, y) {
        console.log("image pos:", x, y)
    }
}
```

C++ 调用示例：

```cpp
display->setImage(image);
display->fitToWindow();
display->addRect("roi_1", QRectF(100, 100, 300, 200));
display->clearGraphics();
```

---

## 2. 技术选型

### 2.1 基础技术栈

- C++17 或 C++20
- Qt 6
- Qt Quick / QML
- CMake
- Windows / MSVC
- DLL 动态链接库

### 2.2 推荐渲染路线

不要用纯 QML `Image` 组件实现工业大图显示。

推荐实现：

```cpp
class VisionDisplayItem : public QQuickItem
```

底层使用：

- `QQuickItem`
- `updatePaintNode()`
- `QSGSimpleTextureNode`
- `QSGTexture`
- `QImage`
- 后续可扩展 OpenGL / RHI / QRhi

原因：

- 适合大图显示
- 适合实时刷新
- 可减少 QML 层负担
- 便于控制鼠标交互与坐标映射
- 便于后续扩展 Overlay 渲染

---

## 3. 工程结构建议

建议项目目录：

```text
VisionDisplayLib/
│
├── CMakeLists.txt
│
├── docs/
│   └── VisionDisplayLib_Spec.md
│
├── include/
│   └── VisionDisplay/
│       ├── VisionDisplay_global.h
│       ├── VisionDisplayItem.h
│       ├── VisionDisplayTypes.h
│       ├── CoordinateMapper.h
│       ├── GraphicObject.h
│       ├── GraphicManager.h
│       ├── RoiObject.h
│       ├── RoiManager.h
│       └── VisionDisplayController.h
│
├── src/
│   ├── VisionDisplayItem.cpp
│   ├── CoordinateMapper.cpp
│   ├── GraphicObject.cpp
│   ├── GraphicManager.cpp
│   ├── RoiObject.cpp
│   ├── RoiManager.cpp
│   ├── VisionDisplayController.cpp
│   └── Plugin.cpp
│
├── qml/
│   └── VisionDisplay.qml
│
└── examples/
    └── DemoApp/
        ├── CMakeLists.txt
        ├── main.cpp
        └── Main.qml
```

---

## 4. DLL 导出要求

Windows 下需要提供导出宏。

文件：

```text
include/VisionDisplay/VisionDisplay_global.h
```

建议内容：

```cpp
#pragma once

#include <QtCore/qglobal.h>

#if defined(VISIONDISPLAY_LIBRARY)
#  define VISIONDISPLAY_API Q_DECL_EXPORT
#else
#  define VISIONDISPLAY_API Q_DECL_IMPORT
#endif
```

所有对外暴露的类必须使用：

```cpp
class VISIONDISPLAY_API VisionDisplayItem : public QQuickItem
{
    Q_OBJECT
};
```

---

## 5. QML 插件要求

需要注册 QML 类型：

```cpp
qmlRegisterType<VisionDisplayItem>("VisionDisplay", 1, 0, "VisionDisplay");
```

外部项目使用：

```qml
import VisionDisplay 1.0

VisionDisplay {
    anchors.fill: parent
}
```

如果使用 `qt_add_qml_module`，需要保证：

- 模块 URI 稳定，例如 `VisionDisplay`
- QML import 名称和 CMake URI 一致
- C++ 类型注册稳定
- Demo 工程可以直接 import 使用

---

## 6. 核心类职责

### 6.1 VisionDisplayItem

核心显示控件，继承自 `QQuickItem`。

职责：

- 接收图像
- 渲染图像
- 管理缩放和平移
- 处理鼠标事件
- 发出鼠标图像坐标信号
- 调用 CoordinateMapper 做坐标转换
- 调用 GraphicManager / RoiManager 渲染图元和 ROI
- 对 QML 暴露属性、方法、信号

建议头文件：

```cpp
class VISIONDISPLAY_API VisionDisplayItem : public QQuickItem
{
    Q_OBJECT

    Q_PROPERTY(double zoom READ zoom WRITE setZoom NOTIFY zoomChanged)
    Q_PROPERTY(double minZoom READ minZoom WRITE setMinZoom NOTIFY minZoomChanged)
    Q_PROPERTY(double maxZoom READ maxZoom WRITE setMaxZoom NOTIFY maxZoomChanged)
    Q_PROPERTY(bool showPixelInfo READ showPixelInfo WRITE setShowPixelInfo NOTIFY showPixelInfoChanged)
    Q_PROPERTY(bool showCrosshair READ showCrosshair WRITE setShowCrosshair NOTIFY showCrosshairChanged)
    Q_PROPERTY(int interactionMode READ interactionMode WRITE setInteractionMode NOTIFY interactionModeChanged)

public:
    explicit VisionDisplayItem(QQuickItem* parent = nullptr);
    ~VisionDisplayItem() override;

    Q_INVOKABLE void fitToWindow();
    Q_INVOKABLE void setZoomAt(double zoom, double viewX, double viewY);
    Q_INVOKABLE QPointF imageToView(const QPointF& imagePoint) const;
    Q_INVOKABLE QPointF viewToImage(const QPointF& viewPoint) const;

    Q_INVOKABLE void clearGraphics();
    Q_INVOKABLE void clearRois();

    Q_INVOKABLE void addLine(const QString& id, double x1, double y1, double x2, double y2);
    Q_INVOKABLE void addRect(const QString& id, double x, double y, double w, double h);
    Q_INVOKABLE void addCircle(const QString& id, double cx, double cy, double r);
    Q_INVOKABLE void addText(const QString& id, double x, double y, const QString& text);

signals:
    void zoomChanged();
    void minZoomChanged();
    void maxZoomChanged();
    void showPixelInfoChanged();
    void showCrosshairChanged();
    void interactionModeChanged();

    void mouseImagePositionChanged(double x, double y);
    void imageClicked(double x, double y);
    void imageDoubleClicked(double x, double y);
    void imageRightClicked(double x, double y);

    void roiCreated(const QString& id);
    void roiChanged(const QString& id);
    void roiSelected(const QString& id);

protected:
    QSGNode* updatePaintNode(QSGNode* oldNode, UpdatePaintNodeData*) override;

    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void geometryChange(const QRectF& newGeometry, const QRectF& oldGeometry) override;
};
```

---

### 6.2 CoordinateMapper

负责坐标变换。

需要管理三类坐标：

- 图像坐标 Image Coordinate
- 控件坐标 View Coordinate
- 世界坐标 / 机械坐标 World Coordinate，后续扩展

基本接口：

```cpp
class CoordinateMapper
{
public:
    void setImageSize(int width, int height);
    void setViewSize(double width, double height);
    void setZoom(double zoom);
    void setOffset(double offsetX, double offsetY);

    QPointF imageToView(const QPointF& imagePoint) const;
    QPointF viewToImage(const QPointF& viewPoint) const;

    QTransform imageToViewTransform() const;
    QTransform viewToImageTransform() const;

    void fitToWindow();

private:
    QSize m_imageSize;
    QSizeF m_viewSize;
    double m_zoom = 1.0;
    QPointF m_offset = QPointF(0.0, 0.0);
};
```

核心关系：

```text
viewX = imageX * zoom + offsetX
viewY = imageY * zoom + offsetY
```

反变换：

```text
imageX = (viewX - offsetX) / zoom
imageY = (viewY - offsetY) / zoom
```

---

### 6.3 GraphicObject

表示 Overlay 图元。

需要支持：

- 点
- 线
- 矩形
- 旋转矩形
- 圆
- 椭圆
- 多边形
- 多段线
- 文本
- 十字线
- 箭头
- 缺陷区域
- 模板匹配轮廓

建议类型：

```cpp
enum class GraphicType {
    Point,
    Line,
    Rect,
    RotatedRect,
    Circle,
    Ellipse,
    Polygon,
    Polyline,
    Text,
    Cross,
    Arrow,
    Contour,
    Region
};
```

图元对象：

```cpp
struct GraphicStyle {
    QColor strokeColor = Qt::green;
    QColor fillColor = Qt::transparent;
    double lineWidth = 1.0;
    bool dashed = false;
    double opacity = 1.0;
    int fontPixelSize = 14;
    bool lineWidthInViewPixels = true;
    bool textSizeInViewPixels = true;
};

class GraphicObject
{
public:
    QString id;
    GraphicType type;
    int layer = 0;
    bool visible = true;
    bool selectable = false;
    GraphicStyle style;

    // 根据 type 存储不同几何数据
};
```

---

### 6.4 GraphicManager

管理所有 Overlay 图元。

职责：

- 添加图元
- 删除图元
- 按 id 查询
- 清空图元
- 清空指定 layer
- 控制 layer 显示隐藏
- 执行 hitTest
- 提供渲染数据

建议接口：

```cpp
class GraphicManager
{
public:
    void addGraphic(const GraphicObject& graphic);
    bool removeGraphic(const QString& id);
    void clear();
    void clearLayer(int layer);

    void setLayerVisible(int layer, bool visible);
    bool isLayerVisible(int layer) const;

    GraphicObject* findById(const QString& id);
    const std::vector<GraphicObject>& graphics() const;

    QString hitTest(const QPointF& imagePoint, double imageTolerance) const;
};
```

---

### 6.5 RoiObject

表示可交互 ROI。

ROI 类型：

```cpp
enum class RoiType {
    Rect,
    RotatedRect,
    Circle,
    Ellipse,
    Polygon,
    Line,
    Caliper
};
```

ROI 基类：

```cpp
class RoiObject
{
public:
    QString id;
    RoiType type;
    QString name;

    bool visible = true;
    bool selected = false;
    bool locked = false;

    QColor strokeColor = Qt::yellow;
    QColor fillColor = QColor(255, 255, 0, 40);

    virtual ~RoiObject() = default;

    virtual QRectF boundingRect() const = 0;
    virtual bool hitTest(const QPointF& imagePoint, double tolerance) const = 0;
    virtual QJsonObject toJson() const = 0;
};
```

旋转矩形 ROI 示例：

```cpp
class RotatedRectRoi : public RoiObject
{
public:
    QPointF center;
    double width = 100.0;
    double height = 60.0;
    double angleDeg = 0.0;

    QRectF boundingRect() const override;
    bool hitTest(const QPointF& imagePoint, double tolerance) const override;
    QJsonObject toJson() const override;
};
```

---

### 6.6 RoiManager

管理 ROI。

职责：

- 创建 ROI
- 删除 ROI
- 选择 ROI
- 修改 ROI
- 导出 JSON
- 命中测试
- 控制 ROI 编辑状态

建议接口：

```cpp
class RoiManager
{
public:
    void addRoi(std::unique_ptr<RoiObject> roi);
    bool removeRoi(const QString& id);
    void clear();

    RoiObject* findById(const QString& id);
    RoiObject* selectedRoi() const;
    void selectRoi(const QString& id);
    void clearSelection();

    QString hitTest(const QPointF& imagePoint, double imageTolerance) const;

    QJsonArray toJson() const;
};
```

---

## 7. 图像输入能力

### 7.1 第一阶段必须支持

```cpp
void setImage(const QImage& image);
```

### 7.2 后续扩展

```cpp
void setImage(const cv::Mat& mat);
void setImage(const uint8_t* data,
              int width,
              int height,
              int stride,
              PixelFormat format);
```

像素格式：

```cpp
enum class PixelFormat {
    Gray8,
    RGB888,
    BGR888,
    RGBA8888,
    BGRA8888
};
```

### 7.3 图像显示要求

必须支持：

- 灰度图
- RGB 图
- RGBA 图
- 大图显示
- 实时刷新
- Fit 显示
- 1:1 显示
- 指定倍率显示

---

## 8. 交互功能要求

### 8.1 缩放

必须支持：

- 鼠标滚轮缩放
- 以鼠标所在图像点为中心缩放
- 设置最小缩放倍率
- 设置最大缩放倍率
- 双击恢复 Fit，可选

滚轮缩放关键规则：

缩放前，鼠标所在 view 坐标对应图像点 P。

缩放后，鼠标所在 view 坐标仍然对应图像点 P。

伪代码：

```cpp
QPointF imageBefore = viewToImage(mouseViewPos);

setZoom(newZoom);

QPointF viewAfter = imageToView(imageBefore);
QPointF delta = mouseViewPos - viewAfter;
offset += delta;
```

---

### 8.2 平移

必须支持：

- 鼠标拖动平移
- 可配置左键 / 中键 / 右键
- 平移时更新 offset
- 平移后触发重绘

---

### 8.3 鼠标图像坐标

鼠标移动时发出：

```cpp
void mouseImagePositionChanged(double x, double y);
```

要求：

- 如果鼠标在图像范围内，输出真实图像坐标
- 如果鼠标在图像范围外，可以继续输出坐标，但需要提供接口判断是否在图像内
- 后续可扩展输出灰度值 / RGB 值

---

## 9. Overlay 图元功能要求

### 9.1 第一阶段支持

- Line
- Rect
- Circle
- Text
- Cross
- Polyline

### 9.2 第二阶段支持

- RotatedRect
- Polygon
- Ellipse
- Arrow
- Contour
- Region

### 9.3 分层要求

建议预定义 layer：

```cpp
enum class DisplayLayer {
    Roi = 10,
    Result = 20,
    Measure = 30,
    Temporary = 40,
    Debug = 50
};
```

需求：

- 可以清除指定 layer
- 可以隐藏指定 layer
- 图元显示顺序按 layer 排序
- ROI 和检测结果不能混在一起管理

---

## 10. ROI 编辑功能要求

### 10.1 第一阶段 ROI

- Rect ROI
- RotatedRect ROI
- Circle ROI
- Line ROI

### 10.2 ROI 交互

必须支持：

- 创建
- 选中
- 拖动
- 缩放
- 删除
- 导出 JSON

后续支持：

- 旋转
- 多边形编辑
- 控制点拖动
- ROI 锁定
- ROI 隐藏
- ROI 复制
- 多选

### 10.3 命中区域

需要定义：

```cpp
enum class HitRegion {
    None,
    Body,
    Center,
    LeftTop,
    RightTop,
    LeftBottom,
    RightBottom,
    LeftEdge,
    RightEdge,
    TopEdge,
    BottomEdge,
    RotateHandle
};
```

注意：

hitTest 的容差应该基于屏幕像素，而不是固定图像像素。

```text
imageTolerance = viewTolerance / zoom
```

---

## 11. 测量功能

后续实现：

- 两点距离
- 点到线距离
- 角度测量
- 圆直径测量
- 灰度剖面
- 卡尺测量
- 显示像素距离 px
- 接入标定后显示实际距离 mm

第一阶段只需要预留接口，不必完整实现。

---

## 12. 图像增强显示

后续实现：

- 亮度调整
- 对比度调整
- Gamma
- 灰度拉伸
- 反色
- 伪彩色
- 直方图
- 放大镜

注意：

图像增强是显示增强，不应修改原始图像数据。

---

## 13. 实时刷新要求

目标场景：

- 工业相机实时显示
- 多相机显示
- 高 FPS 图像刷新

设计要求：

- 采集线程不能直接操作 QML 对象
- UI 渲染必须在 GUI / Scene Graph 相关线程中安全执行
- 避免每帧重复分配大内存
- 后续支持裸指针 buffer 输入
- 后续支持环形缓冲区接入

第一阶段可以接受 `QImage` 深拷贝，但代码结构要为后续零拷贝 / 少拷贝优化预留空间。

---

## 14. 截图与导出

需要支持：

```cpp
bool saveImage(const QString& path);
bool saveScreenshot(const QString& path, bool withOverlay);
bool exportGraphicsJson(const QString& path);
bool exportRoiJson(const QString& path);
```

区别：

- `saveImage`：保存原始图像
- `saveScreenshot`：保存当前显示画面，可选择是否包含 Overlay
- `exportGraphicsJson`：导出图元
- `exportRoiJson`：导出 ROI

---

## 15. QML 暴露属性

建议暴露：

```cpp
Q_PROPERTY(double zoom READ zoom WRITE setZoom NOTIFY zoomChanged)
Q_PROPERTY(double minZoom READ minZoom WRITE setMinZoom NOTIFY minZoomChanged)
Q_PROPERTY(double maxZoom READ maxZoom WRITE setMaxZoom NOTIFY maxZoomChanged)
Q_PROPERTY(bool showCrosshair READ showCrosshair WRITE setShowCrosshair NOTIFY showCrosshairChanged)
Q_PROPERTY(bool showPixelInfo READ showPixelInfo WRITE setShowPixelInfo NOTIFY showPixelInfoChanged)
Q_PROPERTY(int interactionMode READ interactionMode WRITE setInteractionMode NOTIFY interactionModeChanged)
```

QML 示例：

```qml
VisionDisplay {
    id: display

    zoom: 1.0
    minZoom: 0.05
    maxZoom: 100.0
    showCrosshair: true
    showPixelInfo: true
    interactionMode: VisionDisplay.PanMode
}
```

---

## 16. QML 暴露方法

建议暴露：

```cpp
Q_INVOKABLE void fitToWindow();
Q_INVOKABLE void setZoom(double zoom);
Q_INVOKABLE void zoomIn();
Q_INVOKABLE void zoomOut();
Q_INVOKABLE void clearGraphics();
Q_INVOKABLE void clearRois();

Q_INVOKABLE QPointF imageToView(const QPointF& imagePoint) const;
Q_INVOKABLE QPointF viewToImage(const QPointF& viewPoint) const;

Q_INVOKABLE void addLine(const QString& id, double x1, double y1, double x2, double y2);
Q_INVOKABLE void addRect(const QString& id, double x, double y, double w, double h);
Q_INVOKABLE void addCircle(const QString& id, double cx, double cy, double r);
Q_INVOKABLE void addText(const QString& id, double x, double y, const QString& text);
```

---

## 17. QML 暴露信号

建议暴露：

```cpp
void imageClicked(double x, double y);
void imageDoubleClicked(double x, double y);
void imageRightClicked(double x, double y);
void mouseImagePositionChanged(double x, double y);
void zoomChanged(double zoom);

void roiCreated(QString id);
void roiChanged(QString id);
void roiSelected(QString id);

void graphicSelected(QString id);
```

---

## 18. 版本迭代路线

### v0.1：基础图像显示版

目标：

- 实现 `VisionDisplayItem : QQuickItem`
- 支持 `QImage` 显示
- 支持 Fit 显示
- 支持鼠标滚轮缩放
- 支持鼠标拖动平移
- 支持图像坐标 / 控件坐标转换
- 支持鼠标图像坐标输出
- 提供 DemoApp

验收标准：

- Demo 中可以显示一张图像
- 滚轮缩放正常
- 拖动平移正常
- FitToWindow 正常
- 鼠标移动时能输出图像坐标
- 图像缩放时鼠标指向点不漂移

---

### v0.2：Overlay 图元版

目标：

- 实现 GraphicObject
- 实现 GraphicManager
- 支持 Line
- 支持 Rect
- 支持 Circle
- 支持 Text
- 支持 Cross
- 支持 Polyline
- 支持 clearGraphics
- 支持 layer

验收标准：

- 可以在图像坐标上添加图元
- 缩放和平移后图元仍然对准图像
- 线宽不随图像缩放无限变粗
- 可以清除所有图元
- 可以清除指定 layer

---

### v0.3：ROI 编辑版

目标：

- 实现 RoiObject
- 实现 RoiManager
- 支持 Rect ROI
- 支持 RotatedRect ROI
- 支持 Circle ROI
- 支持 Line ROI
- 支持选中 ROI
- 支持拖动 ROI
- 支持删除 ROI
- 支持导出 ROI JSON

验收标准：

- 可以创建 ROI
- 可以选中 ROI
- 可以拖动 ROI
- ROI 缩放和平移后仍然对准图像坐标
- 可以导出 JSON
- 关闭窗口后资源释放正常

---

### v0.4：工业检测结果显示版

目标：

- 支持缺陷框
- 支持缺陷轮廓
- 支持 Blob 区域
- 支持模板匹配轮廓
- 支持拟合线 / 拟合圆显示
- 支持 OK / NG 状态显示

验收标准：

- 检测结果可以按 layer 显示
- 可以单独清除检测结果
- ROI 不会被 clear result 误删

---

### v0.5：性能优化版

目标：

- 支持裸指针图像输入
- 支持 cv::Mat 输入
- 减少图像拷贝
- 优化纹理上传
- 支持实时刷新
- 支持多相机显示

验收标准：

- 3072x2048 图像显示流畅
- 连续刷新不卡顿
- CPU 占用可接受
- 没有明显内存泄漏
- 多个 Display 同时使用不互相影响

---

### v1.0：DLL SDK 版

目标：

- 稳定 DLL 接口
- 稳定 QML import
- 提供 C++ 控制器
- 提供 DemoApp
- 提供基本文档
- 提供版本号
- 提供错误处理

验收标准：

- 其他 Qt/QML 项目可以直接链接 DLL
- QML 可以直接 `import VisionDisplay 1.0`
- DemoApp 可以独立运行
- Release 模式正常
- Debug 模式正常

---

## 19. 最小可用闭环

第一阶段必须优先完成以下 11 个能力：

1. `setImage()`
2. `fitToWindow()`
3. `setZoom()`
4. `zoomIn()`
5. `zoomOut()`
6. 鼠标拖动平移
7. `imageToView()`
8. `viewToImage()`
9. `mouseImagePositionChanged()`
10. `addLine()`
11. `clearGraphics()`

有了这些能力，组件已经可以用于工业检测软件的基本图像显示和结果叠加。

---

## 20. 编码约束

Codex 实现代码时必须遵守：

1. 使用 Qt 6 API。
2. 使用 CMake 构建。
3. 代码应兼容 Windows + MSVC。
4. 对外类放在 `include/VisionDisplay/`。
5. 源文件放在 `src/`。
6. 不要把大量业务逻辑写进 QML。
7. QML 只负责布局和调用。
8. 高性能图像渲染逻辑放在 C++。
9. 坐标转换逻辑必须独立成 `CoordinateMapper`。
10. Overlay 图元和 ROI 必须分开管理。
11. 不要在鼠标事件里写大量临时分配。
12. 不要在每帧渲染时重复创建大量对象。
13. 所有 QML 暴露属性必须有 NOTIFY 信号。
14. DLL 导出类必须使用 `VISIONDISPLAY_API`。
15. DemoApp 必须能直接运行验证功能。

---

## 21. 优先级

### P0：必须实现

- QQuickItem 图像显示
- Fit 显示
- 缩放
- 平移
- 坐标转换
- 鼠标图像坐标
- DemoApp

### P1：重要

- Overlay 图元
- 图元 layer
- clearGraphics
- ROI 基础编辑
- JSON 导出

### P2：增强

- cv::Mat 输入
- 裸指针输入
- 实时刷新优化
- 多相机支持
- 检测结果图元
- 测量工具

### P3：后续扩展

- 图像增强
- 放大镜
- 直方图
- 世界坐标 / 机械坐标
- HALCON HImage 接入
- OpenCV 深度集成

---

## 22. 给 Codex 的实现建议

建议按以下顺序实现，不要一次性实现全部功能：

1. 创建 CMake 工程。
2. 创建 DLL 导出宏。
3. 创建 `VisionDisplayItem`。
4. 注册 QML 类型。
5. 创建 DemoApp。
6. 实现 QImage 显示。
7. 实现 FitToWindow。
8. 实现 CoordinateMapper。
9. 实现鼠标滚轮缩放。
10. 实现鼠标拖动平移。
11. 实现鼠标图像坐标信号。
12. 实现简单 Line Overlay。
13. 实现 GraphicObject / GraphicManager。
14. 扩展 Rect / Circle / Text。
15. 实现 ROI 基础对象。
16. 实现 ROI 选中和拖动。
17. 实现 JSON 导出。
18. 优化实时显示性能。
19. 封装稳定 API。
20. 完善 Demo 和文档。

每完成一个版本，都要保证 DemoApp 可以编译运行。

---

## 23. 重要设计原则

这个项目最关键的不是 UI 外观，而是底层能力。

必须重点保证：

- 图像和图元坐标始终对齐
- 缩放平移不能造成坐标漂移
- 图元和 ROI 使用图像坐标存储
- 图元线宽和文字大小默认使用屏幕像素
- 渲染层和数据层分离
- QML 和 C++ 职责分离
- 后续能支持实时相机刷新
- 后续能封装成稳定 DLL 给别的项目调用

---

## 24. 非目标

当前阶段不要优先做：

- 复杂皮肤主题
- 完整测量系统
- 完整 HALCON 接入
- 完整 OpenCV 接入
- GPU shader 图像增强
- 超复杂 ROI 多选编辑
- 类似 VisionPro 的全部 CogGraphic 类型

先做稳定底座，再逐步迭代。
