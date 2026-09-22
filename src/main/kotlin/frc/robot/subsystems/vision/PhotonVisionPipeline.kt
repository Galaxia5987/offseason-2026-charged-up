package frc.robot.subsystems.vision

import org.photonvision.PhotonCamera


class PhotonVisionPipeline {
    var camera: PhotonCamera = PhotonCamera("YourCameraName")
    fun isDetecting() = camera.allUnreadResults.any {
        it.hasTargets()
    }


    fun getColor() {

        camera.pipelineIndex = 0;
        camera.allUnreadResults.last().bestTarget
    }
}