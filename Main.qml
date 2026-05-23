import QtQuick
import QtQuick.Controls

Window {
    width: 1400
    height: 860
    visible: true
    color: "#202327"
    title: qsTr("Shape Model Debug")

    ShapeModelDebugPage {
        anchors.fill: parent
    }
}
