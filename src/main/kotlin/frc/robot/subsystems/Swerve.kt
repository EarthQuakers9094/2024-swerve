package frc.robot.subsystems

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.apriltag.AprilTagPoseEstimate
import edu.wpi.first.apriltag.AprilTagFields
import frc.robot.Constants
import frc.robot.camera.AprilTagPoseEstimator
import frc.robot.camera.AprilTagResult
import frc.robot.utils.Config
import frc.robot.utils.toNullable
import java.io.File
import java.util.function.Consumer
import kotlin.math.atan2
import swervelib.SwerveController
import swervelib.SwerveDrive
import swervelib.math.SwerveMath
import swervelib.parser.SwerveParser
import swervelib.telemetry.SwerveDriveTelemetry
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator

class Swerve(
// private val camera: PhotonCamera
) : SubsystemBase() {

    var maximumSpeed = Units.feetToMeters(16.0)
    
    var swerveJsonDirectory =
            File(Filesystem.getDeployDirectory(), Config("testswerve", "swerve").config)
    var swerveDrive: SwerveDrive = SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed)

    val robotToCam = Transform3d(Translation3d(0.5, 0.0, 0.5), Rotation3d(0.0, 0.0, 0.0))
    // val poseEstimators = arrayOf(
    //     AprilTagPoseEstimator(swerveDrive, PhotonCamera("ATBack"), Transform3d(-0.3429, 0.0794, 0.2252, Rotation3d(0.0, Math.PI / 6, Math.PI))),
    //     AprilTagPoseEstimator(swerveDrive, PhotonCamera("ATFront"), Transform3d(0.2794, 0.127, 0.2252, Rotation3d(0.0, Math.PI / 6, 0.0))))
        
    //0.0794 meters left of center && 0.3429 meters back from center && 0.2252 meters above && 30 degrees from vertical
    //
    val cameraPoseEstimator = PhotonPoseEstimator(
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(), 
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
        PhotonCamera("ATFront"), 
        Transform3d(0.2794, 0.127, 0.2252, Rotation3d(0.0, Math.PI / 6, 0.0))
)


    // val aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()
    //    val poseEstimator =
    //            PhotonPoseEstimator(
    //                    aprilTagFieldLayout,
    //                    PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
    //                    camera,
    //                    robotToCam
    //            )

    var pdh = PowerDistribution(1, PowerDistribution.ModuleType.kRev)

    private var lastVisionUpdate = -3;

    private var currentUpdate = 0;

    private var visionPose: Pose2d;


    init {
        // poseEstimator.referencePose = Pose3d()

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW
        swerveDrive.setHeadingCorrection(false)

        var getPose = {
            var pose = this.getPos()
            SmartDashboard.putNumber("get pose pathplanner x", pose.getX())
            SmartDashboard.putNumber("get pose pathplanner y", pose.getY())
            SmartDashboard.putNumber("get pose pathplanner rotation", pose.rotation.degrees)

            pose
        }

        var setPose: Consumer<Pose2d> = Consumer { pose -> this.resetOdomentry(pose) }

        var getSpeeds = { this.getSpeeds() }
        var drive: Consumer<ChassisSpeeds> = Consumer { speeds ->
            SmartDashboard.putNumber("path planner direction x", speeds.vxMetersPerSecond)
            SmartDashboard.putNumber("path planner direction y", speeds.vyMetersPerSecond)
            SmartDashboard.putNumber(
                    "path planner direction rotationSpeed",
                    speeds.omegaRadiansPerSecond
            )

            this.driveRobotRel(speeds)
        }

        var replanning = ReplanningConfig()

        var config =
                HolonomicPathFollowerConfig(

                        Constants.Drivebase.TRANSLATION_PID.config,
                        Constants.Drivebase.ROTATION_PID.config,
                        Constants.Drivebase.MAX_AUTO_SPEEDS,
                        Constants.Drivebase.RADIUS,
                        replanning
                )

        var flip = {
            // Boolean supplier that controls when the path will be mirrored for the
            // red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance()
            var ret = false
            if (alliance.isPresent()) {
                ret = alliance.get() == DriverStation.Alliance.Red
            }
            ret
        }

        AutoBuilder.configureHolonomic(
                getPose, // Robot pose supplier
                setPose, // Method to reset odometry (will be called if your auto
                // has a starting pose)
                getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                drive, // Method that will drive the robot given ROBOT RELATIVE
                // ChassisSpeeds
                config,
                flip,
                this // Reference to this subsystem to set requirements
        )
        val driveConversionFactor =
                SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4.15686), 6.12, 1.0)

        SmartDashboard.putNumber("drive conversion factor", driveConversionFactor)

        visionPose = getVisionPose()
    }

    fun checkResetAngles() {
        swerveDrive.checkResetAngles();
    }

    /** This method will be called once per scheduler run */
    override fun periodic() {
        // DriverStation.reportError("YOU SUCK!", true)/
        currentUpdate = currentUpdate + 1;

        if (currentUpdate % 10 == 0) {
            swerveDrive.checkResetAngles()
        }

        // SmartDashboard.putNumber("front left", frontleftCanCoder.getAbsolutePosition().value)
        // SmartDashboard.putNumber("front right", frontrightCanCoder.getAbsolutePosition().value)
        // SmartDashboard.putNumber("back left", backleftCanCoder.getAbsolutePosition().value)
        // SmartDashboard.putNumber("back right", backrightCanCoder.getAbsolutePosition().value)
        SmartDashboard.putNumber("pigeon", swerveDrive.yaw.degrees)

        // SmartDashboard.putNumber("current: backleft", pdh.getCurrent(17))
        // SmartDashboard.putNumber("current: backright", pdh.getCurrent(1))
        // val estimates = poseEstimators.map { it.update() }
        // val sum = estimates.fold(0) {acc, estimate -> 
        //     acc + estimate.targets
        // }
        cameraPoseEstimator.update()//.toNullable()?.estimatedPose?.toPose2d()//?.let { swerveDrive.resetOdometry(it)}
        
        // if(sum >= 0) {
        //     estimates.forEach { estimate ->
        //         estimate.estimatedPose?.let { 
        //             swerveDrive.addVisionMeasurement(   
        //                 it.estimatedPose.toPose2d(),
        //                 estimate.timestamp,
        //                 Constants.Camera.visionSTDEV
        //             )
                
        //         }
        //     }
        // }

        // val sum = results.fold(0) { acc, result ->  
        //     acc + result.targets
        // }
        //     SmartDashboard.putBoolean("valid vision data", true)

        //     results.forEach { res ->
        //         res.estimatedPose?.let {
        //     }
        // } else {
        //     SmartDashboard.putBoolean("valid vision data", false)
        // }


    }

    fun drive(translation: Translation2d, rotation: Double, fieldRelative: Boolean) {
        swerveDrive.drive(
                translation,
                rotation,
                fieldRelative,
                false
        ) // Open loop is disabled since it shouldn't be used most of the time.
    }

    fun drive(velocity: ChassisSpeeds, isOpenLoop: Boolean, centerOfRotationMeters: Translation2d) {
        swerveDrive.drive(velocity, isOpenLoop, centerOfRotationMeters)
    }

    fun driveRobotRel(speeds: ChassisSpeeds) {
        drive(
                Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
                speeds.omegaRadiansPerSecond,
                false
        )
    }

    fun getPos(): Pose2d {
        return swerveDrive.pose
    }

    fun zeroGyro() {
        swerveDrive.zeroGyro()
    }

    fun resetOdomentry(pose: Pose2d) {
        swerveDrive.resetOdometry(pose)
    }

    fun setMotorBrake(enabled: Boolean) {
        swerveDrive.setMotorIdleMode(enabled)
    }

    fun getSwerveController(): SwerveController {
        return swerveDrive.swerveController
    }

    fun getSpeeds(): ChassisSpeeds {
        return swerveDrive.fieldVelocity
    }

    fun getVisionPose(): Pose2d {
        if (lastVisionUpdate < currentUpdate) {
            visionPose = cameraPoseEstimator.update().toNullable()?.estimatedPose?.toPose2d() ?: getPos()
            lastVisionUpdate = currentUpdate
        }

        return getPos()
    }

    fun speakerAngle(): Rotation2d {
        // val poseEstimation = cameraPoseEstimator.update().toNullable()?.estimatedPose?.toPose2d()
        // SmartDashboard.putString("Pose Estimation for speaker angle", poseEstimation?.let { "hello there is a thing here" } ?: "null")
        // val location = poseEstimation ?: getPos()

        val location = getVisionPose()

        val aimingLoc:Double = Constants.Camera.yPositionOfSpeaker;
        // if (Math.abs(location.getY() - Constants.Camera.yPositionOfSpeaker) <= Constants.Camera.offset) {
        //     -location.getY() + 2.0 * Constants.Camera.yPositionOfSpeaker
        // } else if (location.getY() <= Constants.Camera.yPositionOfSpeaker) {
        //     Constants.Camera.yPositionOfSpeaker + Constants.Camera.offset
        // } else {
        //     Constants.Camera.yPositionOfSpeaker - Constants.Camera.offset
        // }

        SmartDashboard.putNumber("aimingloc", aimingLoc)

        SmartDashboard.putNumber("location y aiming", location.getY())

        val ydif = aimingLoc - location.getY()
        val xdif = Constants.Camera.xPositionOfSpeaker() - location.getX()
        SmartDashboard.putNumber("xdif", xdif)
        SmartDashboard.putNumber("ydif", ydif)

        // val xdif = Constants.Camera.xPositionOfSpeaker() - location.getX()


        return Rotation2d.fromRadians(atan2(ydif, xdif))
    }

    fun speakerDistance(): Double {
        // val location = cameraPoseEstimator.update().toNullable()?.estimatedPose?.toPose2d() ?: getPos()
        val location = getVisionPose();

        val aimingLoc:Double = Constants.Camera.yPositionOfSpeaker;

        // if (Math.abs(location.getY() - Constants.Camera.yPositionOfSpeaker) <= Constants.Camera.offset) {
        //     -location.getY() + 2.0 * Constants.Camera.yPositionOfSpeaker
        // } else if (location.getY() <= Constants.Camera.yPositionOfSpeaker) {
        //     Constants.Camera.yPositionOfSpeaker + Constants.Camera.offset
        // } else {
        //     Constants.Camera.yPositionOfSpeaker - Constants.Camera.offset
        // }

        val ydif = aimingLoc - location.getY()
        val xdif = Constants.Camera.xPositionOfSpeaker() - location.getX()
        
        val distance = Math.sqrt(ydif*ydif+xdif*xdif) - 0.2286/2.0;

        SmartDashboard.putNumber("speaker distance", distance);

        return distance;
    }

    /** This method will be called once per scheduler run during simulation */
    override fun simulationPeriodic() {}
}
