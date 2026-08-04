import QtQuick
import QtQuick.Controls
import QtQuick.Dialogs
import VisionDisplay 1.0

Item {
    id: root

    FileDialog {
        id: imageDialog
        title: "Load template image"
        nameFilters: ["Images (*.png *.jpg *.jpeg *.bmp *.tif *.tiff)", "All files (*)"]
        onAccepted: shapeModelDebug.loadTemplateImage(selectedFile)
    }

    FileDialog {
        id: groundTruthDialog
        title: "Load VisionPro ground truth"
        nameFilters: ["JSON (*.json)", "All files (*)"]
        onAccepted: shapeModelDebug.loadVisionProGroundTruth(selectedFile)
    }

    Rectangle {
        id: leftPanel
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        width: 320
        color: "#2c3137"
        border.color: "#4a515a"

        ScrollView {
            anchors.fill: parent
            anchors.margins: 10
            clip: true

            Column {
                width: parent.width
                spacing: 8

                Text {
                    width: parent.width
                    color: "#f1f3f5"
                    font.pixelSize: 16
                    font.bold: true
                    text: "Shape Model"
                }

                Button {
                    width: parent.width
                    text: "Load Template"
                    onClicked: imageDialog.open()
                }

                Row {
                    width: parent.width
                    spacing: 8
                    Button {
                        width: (parent.width - 8) / 2
                        text: "Load GT"
                        onClicked: groundTruthDialog.open()
                    }
                    Button {
                        width: (parent.width - 8) / 2
                        text: "Clear GT"
                        onClicked: shapeModelDebug.clearGroundTruth()
                    }
                }

                Text {
                    width: parent.width
                    color: shapeModelDebug.groundTruthSummary.length > 0 ? "#8bd5ff" : "#88929c"
                    font.pixelSize: 12
                    wrapMode: Text.WordWrap
                    text: shapeModelDebug.groundTruthSummary.length > 0 ? shapeModelDebug.groundTruthSummary : "No VisionPro GT loaded"
                }

                Button {
                    width: parent.width
                    text: "Build Model From ROI"
                    onClicked: shapeModelDebug.buildModelFromRoi()
                }

                Button {
                    width: parent.width
                    text: shapeModelDebug.coarseMatchingRunning ? "Coarse Matching..." : "Run Coarse Match"
                    enabled: !shapeModelDebug.coarseMatchingRunning
                    onClicked: shapeModelDebug.runCoarseMatch()
                }

                Button {
                    width: parent.width
                    text: shapeModelDebug.coarseMatchingRunning ? "PipelineV2 Running..." : "Run PipelineV2"
                    enabled: !shapeModelDebug.coarseMatchingRunning
                    onClicked: shapeModelDebug.runPipelineV2Match()
                }

                Button {
                    width: parent.width
                    text: shapeModelDebug.coarseMatchingRunning ? "ShapeMatch V3 Running..." : "Run ShapeMatch V3"
                    enabled: !shapeModelDebug.coarseMatchingRunning
                    onClicked: shapeModelDebug.runShapeMatchV3()
                }

                Item {
                    id: coarseProgress
                    width: parent.width
                    height: 78
                    property bool running: shapeModelDebug.coarseMatchingRunning
                    property bool finished: shapeModelDebug.coarseMatchFinished
                    property real spinAngle: 0

                    Timer {
                        interval: 33
                        running: coarseProgress.running
                        repeat: true
                        onTriggered: {
                            coarseProgress.spinAngle = (coarseProgress.spinAngle + 8) % 360
                            progressCanvas.requestPaint()
                        }
                    }

                    onRunningChanged: progressCanvas.requestPaint()
                    onFinishedChanged: progressCanvas.requestPaint()

                    Canvas {
                        id: progressCanvas
                        width: 58
                        height: 58
                        anchors.left: parent.left
                        anchors.leftMargin: 2
                        anchors.verticalCenter: parent.verticalCenter

                        onPaint: {
                            const ctx = getContext("2d")
                            ctx.reset()
                            ctx.clearRect(0, 0, width, height)
                            const cx = width / 2
                            const cy = height / 2
                            const radius = 23
                            ctx.lineCap = "round"
                            ctx.lineWidth = 5
                            ctx.strokeStyle = "#56606a"
                            ctx.beginPath()
                            ctx.arc(cx, cy, radius, 0, Math.PI * 2)
                            ctx.stroke()

                            if (coarseProgress.finished) {
                                ctx.strokeStyle = "#22c55e"
                                ctx.beginPath()
                                ctx.arc(cx, cy, radius, -Math.PI / 2, Math.PI * 1.5)
                                ctx.stroke()
                            } else if (coarseProgress.running) {
                                ctx.strokeStyle = "#5ac8fa"
                                const start = (coarseProgress.spinAngle - 90) * Math.PI / 180
                                ctx.beginPath()
                                ctx.arc(cx, cy, radius, start, start + Math.PI * 1.35)
                                ctx.stroke()
                            } else {
                                ctx.strokeStyle = "#7b858f"
                                ctx.beginPath()
                                ctx.arc(cx, cy, radius, -Math.PI / 2, -Math.PI / 2 + Math.PI * 0.18)
                                ctx.stroke()
                            }
                        }
                    }

                    Text {
                        anchors.left: progressCanvas.right
                        anchors.leftMargin: 12
                        anchors.verticalCenter: parent.verticalCenter
                        width: parent.width - progressCanvas.width - 18
                        color: coarseProgress.finished ? "#22c55e" : (coarseProgress.running ? "#5ac8fa" : "#cfd6dd")
                        font.pixelSize: 14
                        font.bold: coarseProgress.finished
                        text: coarseProgress.finished ? "finished" : (coarseProgress.running ? "running" : "ready")
                    }
                }

                Row {
                    width: parent.width
                    spacing: 8
                    Button {
                        width: (parent.width - 8) / 2
                        text: "Enable Model ROI"
                        onClicked: {
                            shapeModelDebug.modelRoiVisible = true
                            shapeModelDebug.modelRoiEditable = true
                        }
                    }
                    Button {
                        width: (parent.width - 8) / 2
                        text: "Reset Model ROI"
                        onClicked: shapeModelDebug.resetModelRoi()
                    }
                }

                Button {
                    width: parent.width
                    text: "Save Debug Images"
                    onClicked: shapeModelDebug.saveDebugImages()
                }

                Row {
                    width: parent.width
                    spacing: 8
                    Text {
                        width: 90
                        color: "#cfd6dd"
                        text: "Level"
                        anchors.verticalCenter: parent.verticalCenter
                    }
                    SpinBox {
                        from: 0
                        to: Math.max(0, shapeModelDebug.levelCount - 1)
                        value: shapeModelDebug.currentLevel
                        editable: true
                        onValueModified: shapeModelDebug.setCurrentLevel(value)
                    }
                }

                Rectangle { width: parent.width; height: 1; color: "#4a515a" }

                CheckBox {
                    text: "Show Model ROI"
                    checked: shapeModelDebug.modelRoiVisible
                    onToggled: shapeModelDebug.modelRoiVisible = checked
                    palette.text: "#cfd6dd"
                }

                CheckBox {
                    text: "Lock ROI"
                    checked: !shapeModelDebug.modelRoiEditable
                    onToggled: shapeModelDebug.modelRoiEditable = !checked
                    palette.text: "#cfd6dd"
                }

                Text {
                    width: parent.width
                    color: "#cfd6dd"
                    font.family: "Consolas"
                    font.pixelSize: 12
                    wrapMode: Text.WordWrap
                    text: "ROI x=" + shapeModelDebug.modelRoi.x.toFixed(1)
                        + " y=" + shapeModelDebug.modelRoi.y.toFixed(1)
                        + " w=" + shapeModelDebug.modelRoi.width.toFixed(1)
                        + " h=" + shapeModelDebug.modelRoi.height.toFixed(1)
                }

                Row {
                    width: parent.width
                    spacing: 8
                    Button {
                        width: (parent.width - 8) / 2
                        text: "Enable Search ROI"
                        onClicked: shapeModelDebug.enableSearchRoi()
                    }
                    Button {
                        width: (parent.width - 8) / 2
                        text: "Reset Search ROI"
                        onClicked: shapeModelDebug.resetSearchRoi()
                    }
                }

                CheckBox {
                    text: "Limit Search ROI"
                    checked: shapeModelDebug.searchRoiEnabled
                    onToggled: shapeModelDebug.searchRoiEnabled = checked
                    palette.text: "#cfd6dd"
                }

                Text {
                    width: parent.width
                    color: shapeModelDebug.searchRoiEnabled ? "#ffe08a" : "#88929c"
                    font.family: "Consolas"
                    font.pixelSize: 12
                    wrapMode: Text.WordWrap
                    text: "Search x=" + shapeModelDebug.searchRoi.x.toFixed(1)
                        + " y=" + shapeModelDebug.searchRoi.y.toFixed(1)
                        + " w=" + shapeModelDebug.searchRoi.width.toFixed(1)
                        + " h=" + shapeModelDebug.searchRoi.height.toFixed(1)
                }

                Rectangle { width: parent.width; height: 1; color: "#4a515a" }

                CheckBox { text: "Chains"; checked: shapeModelDebug.showChains; onToggled: shapeModelDebug.showChains = checked; palette.text: "#cfd6dd" }
                CheckBox { text: "Sampled points"; checked: shapeModelDebug.showSampledPoints; onToggled: shapeModelDebug.showSampledPoints = checked; palette.text: "#cfd6dd" }
                CheckBox { text: "Normals"; checked: shapeModelDebug.showNormals; onToggled: shapeModelDebug.showNormals = checked; palette.text: "#cfd6dd" }
                CheckBox { text: "Tangents"; checked: shapeModelDebug.showTangents; onToggled: shapeModelDebug.showTangents = checked; palette.text: "#cfd6dd" }
                CheckBox { text: "Curvature / corners"; checked: shapeModelDebug.showCurvature; onToggled: shapeModelDebug.showCurvature = checked; palette.text: "#cfd6dd" }
                CheckBox { text: "Rejected points"; checked: shapeModelDebug.showRejectedPoints; onToggled: shapeModelDebug.showRejectedPoints = checked; palette.text: "#cfd6dd" }
                CheckBox { text: "All edges"; checked: shapeModelDebug.showAllEdges; onToggled: shapeModelDebug.showAllEdges = checked; palette.text: "#cfd6dd" }
                CheckBox { text: "Stable points"; checked: shapeModelDebug.showStablePoints; onToggled: shapeModelDebug.showStablePoints = checked; palette.text: "#cfd6dd" }
                CheckBox { text: "Chain id"; checked: shapeModelDebug.showChainId; onToggled: shapeModelDebug.showChainId = checked; palette.text: "#cfd6dd" }
                CheckBox { text: "Point id"; checked: shapeModelDebug.showPointId; onToggled: shapeModelDebug.showPointId = checked; palette.text: "#cfd6dd" }

                Row {
                    width: parent.width
                    spacing: 8
                    Text { width: 130; color: "#cfd6dd"; text: "normal step"; anchors.verticalCenter: parent.verticalCenter }
                    SpinBox {
                        from: 1
                        to: 128
                        value: shapeModelDebug.normalStep
                        editable: true
                        onValueModified: shapeModelDebug.normalStep = value
                    }
                }

                Row {
                    width: parent.width
                    spacing: 8
                    Text { width: 130; color: "#cfd6dd"; text: "normal length"; anchors.verticalCenter: parent.verticalCenter }
                    SpinBox {
                        from: 1
                        to: 80
                        value: Math.round(shapeModelDebug.normalLength)
                        editable: true
                        onValueModified: shapeModelDebug.normalLength = value
                    }
                }

                Row {
                    width: parent.width
                    spacing: 8
                    Text { width: 130; color: "#cfd6dd"; text: "tangent step"; anchors.verticalCenter: parent.verticalCenter }
                    SpinBox {
                        from: 1
                        to: 128
                        value: shapeModelDebug.tangentStep
                        editable: true
                        onValueModified: shapeModelDebug.tangentStep = value
                    }
                }

                Row {
                    width: parent.width
                    spacing: 8
                    Text { width: 130; color: "#cfd6dd"; text: "tangent length"; anchors.verticalCenter: parent.verticalCenter }
                    SpinBox {
                        from: 1
                        to: 80
                        value: Math.round(shapeModelDebug.tangentLength)
                        editable: true
                        onValueModified: shapeModelDebug.tangentLength = value
                    }
                }

                Rectangle { width: parent.width; height: 1; color: "#4a515a" }

                Text {
                    width: parent.width
                    color: "#cfe8ff"
                    font.pixelSize: 12
                    wrapMode: Text.WordWrap
                    text: shapeModelDebug.status
                }
            }
        }
    }

    VisionDisplay {
        id: display
        anchors.left: leftPanel.right
        anchors.right: rightPanel.left
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        minZoom: 0.02
        maxZoom: 100.0
        interactionMode: VisionDisplay.PanMode
        showCrosshair: true
        modelRoi: shapeModelDebug.modelRoi
        modelRoiVisible: shapeModelDebug.modelRoiVisible
        modelRoiEditable: shapeModelDebug.modelRoiEditable

        Component.onCompleted: shapeModelDebug.bindDisplay(display)

        onModelRoiChanged: {
            if (shapeModelDebug.modelRoi !== modelRoi) {
                shapeModelDebug.modelRoi = modelRoi
            }
        }
    }

    Rectangle {
        id: rightPanel
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        width: 300
        color: "#252a30"
        border.color: "#4a515a"

        ScrollView {
            anchors.fill: parent
            anchors.margins: 10
            clip: true
            Text {
                width: parent.width
                color: "#dce7f2"
                font.family: "Consolas"
                font.pixelSize: 12
                wrapMode: Text.WordWrap
                text: shapeModelDebug.statisticsText
            }
        }
    }
}
