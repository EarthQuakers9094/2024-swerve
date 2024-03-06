package frc.robot.camera

import edu.wpi.first.hal.HALUtil
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants
import frc.robot.subsystems.Swerve
import frc.robot.utils.toNullable
import frc.robot.utils.toPose3d
import frc.robot.utils.debug
import frc.robot.camera.AprilTagResult
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.EstimatedRobotPose
import swervelib.SwerveDrive

class AprilTagPoseEstimator(val swerve: SwerveDrive, val camera: PhotonCamera, private val offset: Transform3d) {

    private val poseEstimator = PhotonPoseEstimator(
        Constants.Camera.aprilTagFieldLayout,
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        camera,
        if(!Constants.Camera.cameraOfffsetEstimation) {offset} else {Transform3d(0.0, 0.0, 0.0, Rotation3d(0.0, 0.0, 0.0))}
    )



    init {
        poseEstimator.referencePose = swerve.pose.toPose3d()
    }

    fun update() {
        if (!camera.isConnected) {
            DriverStation.reportWarning("Camera with name: ${camera.name} is not connected", false)
            return// AprilTagResult(null, 0)
        }
        if(Constants.Camera.cameraOfffsetEstimation) {
            poseEstimator.update().toNullable()?.let { 
                SmartDashboard.putString("[${camera.name}] estimated offset", it.estimatedPose.debug())
            }
            return
        }

        if(camera.latestResult.targets.size >= 2) {
            poseEstimator.update().toNullable()?.let { 
                swerve.addVisionMeasurement(
                    it.estimatedPose.toPose2d(),
                    Timer.getFPGATimestamp(),
                    Constants.Camera.visionSTDEV)
            }
        }
        return

    }
        
        // AprilTagResult(, camera.latestResult.targets.size)/*?.let {   
                
    
    
}
