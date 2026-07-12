import QtQuick
import QtQuick3D
import QtQuick3D.Helpers
import QtQuick3D.AssetUtils

Item {
    id: root

    property url modelSource
    property real rollAngle: 0
    property real pitchAngle: 0
    property real yawAngle: 0
    property real modelScale: 1.0

    View3D {
        anchors.fill: parent

        environment: SceneEnvironment {
            clearColor: "#111820"
            backgroundMode: SceneEnvironment.Color
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.High
        }

        PerspectiveCamera {
            id: camera
            position: Qt.vector3d(0, 160, 420)
            eulerRotation: Qt.vector3d(-18, 0, 0)
            clipFar: 5000
        }

        DirectionalLight {
            eulerRotation: Qt.vector3d(-45, 35, 0)
            brightness: 1.5
        }

        PointLight {
            position: Qt.vector3d(0, 260, 260)
            brightness: 35
        }

        RuntimeLoader {
            id: robotModel
            objectName: "robotModel"
            source: root.modelSource
            scale: Qt.vector3d(root.modelScale, root.modelScale, root.modelScale)
            eulerRotation: Qt.vector3d(root.pitchAngle, root.yawAngle, root.rollAngle)
        }

        Model {
            source: "#Grid"
            y: -120
            scale: Qt.vector3d(5, 1, 5)
            materials: PrincipledMaterial {
                baseColor: "#26323d"
                alphaMode: PrincipledMaterial.Blend
                opacity: 0.28
            }
        }

        OrbitCameraController {
            camera: camera
            origin: robotModel
        }
    }

    Rectangle {
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.margins: 10
        width: statusText.width + 18
        height: statusText.height + 12
        color: "#aa101418"
        border.color: "#2b3541"
        radius: 4

        Text {
            id: statusText
            anchors.centerIn: parent
            color: "#d6dde6"
            text: "GLB | roll " + root.rollAngle.toFixed(1) +
                  " pitch " + root.pitchAngle.toFixed(1) +
                  " yaw " + root.yawAngle.toFixed(1) +
                  " | " + (robotModel.status === RuntimeLoader.Success ? "loaded" :
                           robotModel.status === RuntimeLoader.Error ? robotModel.errorString : "loading")
            font.pixelSize: 13
        }
    }
}
