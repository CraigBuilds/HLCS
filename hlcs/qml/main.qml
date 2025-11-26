import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

ApplicationWindow {
    visible: true
    width: 600
    height: 400
    title: "HLCS - High Level Control System"

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 20
        spacing: 20

        // Title
        Label {
            text: "HLCS Control Panel"
            font.pixelSize: 24
            font.bold: true
            Layout.alignment: Qt.AlignHCenter
        }

        // Data display section
        GroupBox {
            title: "Live Data"
            Layout.fillWidth: true
            
            GridLayout {
                anchors.fill: parent
                columns: 2
                rowSpacing: 10
                columnSpacing: 20

                Label {
                    text: "Data Value:"
                    font.pixelSize: 16
                }
                Label {
                    id: dataValueLabel
                    text: bridge.dataValue.toFixed(2)
                    font.pixelSize: 16
                    font.bold: true
                    color: "#2196F3"
                }

                Label {
                    text: "Counter:"
                    font.pixelSize: 16
                }
                Label {
                    id: counterValueLabel
                    text: bridge.counterValue.toString()
                    font.pixelSize: 16
                    font.bold: true
                    color: "#4CAF50"
                }
            }
        }

        // Control buttons section
        GroupBox {
            title: "Controls"
            Layout.fillWidth: true
            
            RowLayout {
                anchors.fill: parent
                spacing: 10

                Button {
                    text: "Increment Counter"
                    Layout.fillWidth: true
                    onClicked: bridge.incrementCounter()
                    background: Rectangle {
                        color: parent.pressed ? "#1976D2" : "#2196F3"
                        radius: 4
                    }
                    contentItem: Text {
                        text: parent.text
                        color: "white"
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                }

                Button {
                    text: "Reset Counter"
                    Layout.fillWidth: true
                    onClicked: bridge.resetCounter()
                    background: Rectangle {
                        color: parent.pressed ? "#C62828" : "#F44336"
                        radius: 4
                    }
                    contentItem: Text {
                        text: parent.text
                        color: "white"
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                }
            }
        }

        // Status section
        GroupBox {
            title: "Status"
            Layout.fillWidth: true
            Layout.fillHeight: true
            
            ScrollView {
                anchors.fill: parent
                
                TextArea {
                    id: statusText
                    text: bridge.statusMessage
                    readOnly: true
                    wrapMode: Text.Wrap
                    font.pixelSize: 12
                }
            }
        }

        // Info label
        Label {
            text: "HLCS v0.0.1 - High Level Control System"
            font.pixelSize: 10
            color: "#888888"
            Layout.alignment: Qt.AlignHCenter
        }
    }
}
