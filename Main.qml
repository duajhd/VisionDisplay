import QtQuick
import QtQuick.Controls
import QtQuick.Dialogs
import VisionDisplay 1.0

Window {
    width: 1200
    height: 800
    visible: true
    color: "#202327"
    title: qsTr("VisionDisplay - Single Caliper Debug")

    property string singleCaliperToolId: "SingleCaliper_1"

    function singleCaliperNumber(field, fallbackValue) {
        const value = Number(field.text)
        return Number.isFinite(value) ? value : fallbackValue
    }

    function drawSingleCaliperPreview() {
        const centerX = singleCaliperNumber(singleCaliperCenterX, 656.0)
        const centerY = singleCaliperNumber(singleCaliperCenterY, 320.0)
        const searchLength = Math.max(1.0, singleCaliperNumber(singleCaliperSearchLength, 120.0))
        const projectionWidth = Math.max(0.0, singleCaliperNumber(singleCaliperProjectionWidth, 48.0))
        const directionDeg = singleCaliperNumber(singleCaliperDirection, 0.0)

        display.clearAllToolGraphics()
        display.createEditableSingleCaliper(singleCaliperToolId,
                                            centerX,
                                            centerY,
                                            projectionWidth,
                                            searchLength,
                                            directionDeg)
    }

    function currentSingleCaliperGeometry() {
        let geometry = display.editableCaliperGeometry(singleCaliperToolId)
        if (!geometry.valid || geometry.type !== "single") {
            drawSingleCaliperPreview()
            geometry = display.editableCaliperGeometry(singleCaliperToolId)
        }
        return geometry
    }

    function resetSyntheticImage() {
        integratedController.bindDisplay(display)
        drawSingleCaliperPreview()
        singleCaliperDiagnostics.text = integratedController.status
    }

    VisionDisplay {
        id: display
        objectName: "display"
        anchors.fill: parent
        minZoom: 0.02
        maxZoom: 100.0
        interactionMode: VisionDisplay.PanMode
        autoFitOnNewImage: true
        keepViewTransformOnNewImage: true

        Component.onCompleted: {
            resetSyntheticImage()
        }

        onMouseImagePositionChanged: function(x, y) {
            cursorText.text = "x: " + x.toFixed(1) + "  y: " + y.toFixed(1)
                + "  zoom: " + zoom.toFixed(3)
        }
    }

    FileDialog {
        id: imageDialog
        title: "Load grayscale image"
        nameFilters: ["Images (*.png *.jpg *.jpeg *.bmp *.tif *.tiff)", "All files (*)"]
        onAccepted: {
            const path = selectedFile.toString()
            const controllerLoaded = integratedController.loadImage(path)
            const displayLoaded = display.loadImage(path)
            if (displayLoaded) {
                display.fitToWindow()
                drawSingleCaliperPreview()
            }
            singleCaliperDiagnostics.text = controllerLoaded && displayLoaded
                ? integratedController.status
                : "Load image failed"
        }
    }

    Rectangle {
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        width: 380
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
                    text: "Single Caliper Debug"
                }

                Button {
                    width: parent.width
                    text: "Load Image"
                    onClicked: imageDialog.open()
                }

                Button {
                    width: parent.width
                    text: "Reset Synthetic Image"
                    onClicked: {
                        resetSyntheticImage()
                    }
                }

                Rectangle { width: parent.width; height: 1; color: "#4a515a" }

                Row {
                    width: parent.width
                    spacing: 6
                    Text { width: 126; color: "#cfd6dd"; font.pixelSize: 12; text: "centerX"; anchors.verticalCenter: parent.verticalCenter }
                    TextField {
                        id: singleCaliperCenterX
                        width: parent.width - 132
                        text: "656.0"
                        inputMethodHints: Qt.ImhFormattedNumbersOnly
                        onEditingFinished: drawSingleCaliperPreview()
                    }
                }

                Row {
                    width: parent.width
                    spacing: 6
                    Text { width: 126; color: "#cfd6dd"; font.pixelSize: 12; text: "centerY"; anchors.verticalCenter: parent.verticalCenter }
                    TextField {
                        id: singleCaliperCenterY
                        width: parent.width - 132
                        text: "320.0"
                        inputMethodHints: Qt.ImhFormattedNumbersOnly
                        onEditingFinished: drawSingleCaliperPreview()
                    }
                }

                Row {
                    width: parent.width
                    spacing: 6
                    Text { width: 126; color: "#cfd6dd"; font.pixelSize: 12; text: "searchLength"; anchors.verticalCenter: parent.verticalCenter }
                    TextField {
                        id: singleCaliperSearchLength
                        width: parent.width - 132
                        text: "120.0"
                        inputMethodHints: Qt.ImhFormattedNumbersOnly
                        onEditingFinished: drawSingleCaliperPreview()
                    }
                }

                Row {
                    width: parent.width
                    spacing: 6
                    Text { width: 126; color: "#cfd6dd"; font.pixelSize: 12; text: "projectionWidth"; anchors.verticalCenter: parent.verticalCenter }
                    TextField {
                        id: singleCaliperProjectionWidth
                        width: parent.width - 132
                        text: "48.0"
                        inputMethodHints: Qt.ImhFormattedNumbersOnly
                        onEditingFinished: drawSingleCaliperPreview()
                    }
                }

                Row {
                    width: parent.width
                    spacing: 6
                    Text { width: 126; color: "#cfd6dd"; font.pixelSize: 12; text: "directionDeg"; anchors.verticalCenter: parent.verticalCenter }
                    TextField {
                        id: singleCaliperDirection
                        width: parent.width - 132
                        text: "0.0"
                        inputMethodHints: Qt.ImhFormattedNumbersOnly
                        onEditingFinished: drawSingleCaliperPreview()
                    }
                }

                Row {
                    width: parent.width
                    spacing: 6
                    Text { width: 126; color: "#cfd6dd"; font.pixelSize: 12; text: "polarity"; anchors.verticalCenter: parent.verticalCenter }
                    ComboBox {
                        id: singleCaliperPolarity
                        width: parent.width - 132
                        model: ["DarkToLight", "LightToDark", "Any"]
                    }
                }

                Row {
                    width: parent.width
                    spacing: 6
                    Text { width: 126; color: "#cfd6dd"; font.pixelSize: 12; text: "edgeSelection"; anchors.verticalCenter: parent.verticalCenter }
                    ComboBox {
                        id: singleCaliperEdgeSelection
                        width: parent.width - 132
                        model: ["Strongest", "First", "Last", "NearestToCenter", "NearestToExpected", "All"]
                    }
                }

                Row {
                    width: parent.width
                    spacing: 6
                    Text { width: 126; color: "#cfd6dd"; font.pixelSize: 12; text: "minResponse"; anchors.verticalCenter: parent.verticalCenter }
                    TextField { id: singleCaliperMinResponse; width: parent.width - 132; text: "5.0"; inputMethodHints: Qt.ImhFormattedNumbersOnly }
                }

                Row {
                    width: parent.width
                    spacing: 6
                    Text { width: 126; color: "#cfd6dd"; font.pixelSize: 12; text: "projectionCount"; anchors.verticalCenter: parent.verticalCenter }
                    TextField { id: singleCaliperProjectionCount; width: parent.width - 132; text: "9"; inputMethodHints: Qt.ImhDigitsOnly }
                }

                Row {
                    width: parent.width
                    spacing: 6
                    Text { width: 126; color: "#cfd6dd"; font.pixelSize: 12; text: "smoothingSigma"; anchors.verticalCenter: parent.verticalCenter }
                    TextField { id: singleCaliperSmoothingSigma; width: parent.width - 132; text: "1.0"; inputMethodHints: Qt.ImhFormattedNumbersOnly }
                }

                Row {
                    width: parent.width
                    spacing: 6
                    Text { width: 126; color: "#cfd6dd"; font.pixelSize: 12; text: "expectedPos1D"; anchors.verticalCenter: parent.verticalCenter }
                    TextField { id: singleCaliperExpectedPosition; width: parent.width - 132; text: "0.0"; inputMethodHints: Qt.ImhFormattedNumbersOnly }
                }

                Row {
                    width: parent.width
                    spacing: 6
                    Text { width: 126; color: "#cfd6dd"; font.pixelSize: 12; text: "maxPosDev"; anchors.verticalCenter: parent.verticalCenter }
                    TextField { id: singleCaliperMaxPositionDeviation; width: parent.width - 132; text: "10.0"; inputMethodHints: Qt.ImhFormattedNumbersOnly }
                }

                Row {
                    width: parent.width
                    spacing: 6
                    Text { width: 126; color: "#cfd6dd"; font.pixelSize: 12; text: "fallback"; anchors.verticalCenter: parent.verticalCenter }
                    ComboBox {
                        id: singleCaliperFallbackSelection
                        width: parent.width - 132
                        model: ["Strongest", "First", "Last", "NearestToCenter", "NearestToExpected", "All"]
                    }
                }

                CheckBox {
                    id: singleCaliperShowLabels
                    width: parent.width
                    text: "Show candidate response / position"
                    checked: true
                    palette.text: "#cfd6dd"
                }

                Button {
                    width: parent.width
                    text: "Show Single Caliper"
                    onClicked: {
                        drawSingleCaliperPreview()
                        singleCaliperDiagnostics.text = "SingleCaliper preview updated"
                    }
                }

                Button {
                    width: parent.width
                    text: "Run Single Caliper"
                    onClicked: {
                        const geometry = currentSingleCaliperGeometry()
                        if (!geometry.valid || geometry.type !== "single") {
                            singleCaliperDiagnostics.text = "SingleCaliper is not ready"
                            return
                        }

                        const result = integratedController.runSingleCaliperDebug(
                            geometry.centerX,
                            geometry.centerY,
                            geometry.height,
                            geometry.width,
                            geometry.searchDirectionAngleDeg,
                            singleCaliperPolarity.currentIndex,
                            singleCaliperEdgeSelection.currentIndex,
                            singleCaliperNumber(singleCaliperMinResponse, 5.0),
                            Math.max(1, Math.round(singleCaliperNumber(singleCaliperProjectionCount, 9))),
                            singleCaliperNumber(singleCaliperSmoothingSigma, 1.0),
                            singleCaliperNumber(singleCaliperExpectedPosition, 0.0),
                            singleCaliperNumber(singleCaliperMaxPositionDeviation, 10.0),
                            singleCaliperFallbackSelection.currentIndex)
                        let candidates = result.candidates
                        if (!candidates || candidates.length === undefined) {
                            candidates = JSON.parse(result.candidatesJson || "[]")
                        }
                        display.clearAllToolGraphics()
                        display.createEditableSingleCaliper(singleCaliperToolId,
                                                            result.centerX,
                                                            result.centerY,
                                                            result.projectionWidth,
                                                            result.searchLength,
                                                            result.searchDirectionAngleDeg)
                        display.addCaliperEdgePoints(singleCaliperToolId,
                                                     candidates,
                                                     result.selectedIndex,
                                                     result.status,
                                                     singleCaliperShowLabels.checked)
                        if (!result.hasSelected) {
                            display.addToolStatusText(singleCaliperToolId,
                                                      result.centerX + result.projectionWidth * 0.5,
                                                      result.centerY - result.searchLength * 0.5,
                                                      result.status + " - no edge found",
                                                      false)
                        }
                        singleCaliperDiagnostics.text = integratedController.lastSingleCaliperDiagnostics()
                    }
                }

                Button {
                    width: parent.width
                    text: "Clear Single Caliper Graphics"
                    onClicked: {
                        display.clearToolGraphics(singleCaliperToolId)
                        singleCaliperDiagnostics.text = ""
                    }
                }

                Rectangle { width: parent.width; height: 1; color: "#4a515a" }

                Row {
                    width: parent.width
                    spacing: 6

                    Button {
                        width: (parent.width - 12) / 3
                        text: "Fit"
                        onClicked: display.fitToWindow()
                    }

                    Button {
                        width: (parent.width - 12) / 3
                        text: "+"
                        onClicked: display.zoomIn()
                    }

                    Button {
                        width: (parent.width - 12) / 3
                        text: "-"
                        onClicked: display.zoomOut()
                    }
                }

                Item { width: parent.width; height: 8 }

                Text {
                    id: cursorText
                    width: parent.width
                    color: "#f1f3f5"
                    wrapMode: Text.WordWrap
                    text: "x: 0.0  y: 0.0  zoom: 1.000"
                }

                Text {
                    id: singleCaliperDiagnostics
                    width: parent.width
                    color: "#cfe8ff"
                    font.pixelSize: 12
                    font.family: "Consolas"
                    wrapMode: Text.WordWrap
                    text: integratedController.status
                }
            }
        }
    }
}
