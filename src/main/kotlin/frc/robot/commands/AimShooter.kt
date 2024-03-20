package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Swerve
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import kotlin.math.atan2
import kotlin.math.pow
import swervelib.SwerveDrive
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard


class AimShooter( private val shooter: Shooter, private val swerveDrive: Swerve, private val terminate: Boolean, private val camera: PhotonCamera, private val shoot: () -> Boolean) : Command() {
    var height = Constants.Camera.shootElevation;
    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(shooter)
    }

    private enum class ShootingState {
        Idle,
        Revving,
        Shooting,
    }
    private var shootingState = ShootingState.Idle

    override fun initialize() {
        SmartDashboard.putNumber("aiming height", height);
        SmartDashboard.putBoolean("aim shooter running", true);
    }


    override fun execute() {
        SmartDashboard.putString("current aim shooter state", shootingState.toString())
        //height = SmartDashboard.getNumber("aiming height", height);
        
        // val ydif = Constants.Camera.yPositionOfSpeaker-location.getY();
        // val xdif = Constants.Camera.xPositionOfSpeaker()-location.getX();

        // val distance = Math.sqrt( ydif*ydif + 
        //                           xdif*xdif );
        val result = camera.latestResult
        
        val distance = result.targets.filter { it.fiducialId == 4 }.getOrNull(0)?.pitch//swerveDrive.speakerDistance()
        SmartDashboard.putNumber("tag pitch", distance ?: -1.0)
        SmartDashboard.putBoolean("calculating angle for aiming", false)
        distance?.let { x ->
            // v1 val angle = -0.978752 + (8.42889 * distance) - (14.1454 * distance.pow(2)) + (8.09307 * (distance.pow(3)))//atan2(Constants.Camera.shootElevation,distance) - 3.5 * Math.PI/180.0;
            // v2 val angle = 1.83801 - (18.1636 * distance) +( 75.4928 *  distance.pow(2)) - (118.954 * distance.pow(3)) + (63.5236 * distance.pow(4))
            // v3 val angle = 2.3925 - (26.171 * distance) + (119.313 * distance.pow(2)) - (232.193 *  distance.pow(3)) + (201.078 * distance.pow(4)) - (62.5248 * distance.pow(5))
            // v4 val angle = 4.26707 - (46.9928 * distance) + (208.464 * distance.pow(2)) - (416.168 * distance.pow(3)) + (384.272 * distance.pow(4)) - (132.984 * distance.pow(5))
            // v5 val angle = 7.50023 - (189.565 * x) + (1999.42 * x.pow(2)) - (10831.8 * x.pow(3)) + (32686.6 * x.pow(4)) - (55084.5 * x.pow(5)) + (48226.9 * x.pow(6)) - (17006.9 * x.pow(7))
            // v6 val angle = 0.620003 + (0.0107606 * x) + (0.0010628 * x.pow(2)) + (0.000179998 * x.pow(3)) - (0.0000583219 * x.pow(4)) + (7.04788 * (10.0).pow(-6) * x.pow(5)) - (2.63916 * (10.0).pow(-7) * x.pow(6))
            val angle = 0.57444 + (0.0217394 * x)
           
            //0.83, 0.9
            //0.53, 0.72 
            //0.36, 0.6
            //0.28,0.45
            //     ///////////////////0.27, 0.424617 // untested please test
            //0.20, 0.45
            //0.60, 0.686734

            //0.15 ,0.38
            //0.13, 0.362000
            
            SmartDashboard.putBoolean("calculating angle for aiming", true)
            SmartDashboard.putNumber("calculated angle for aiming", angle)
            // both are in radians? https://kotlinlang.org/api/latest/jvm/stdlib/kotlin.math/atan2.html#:~:text=Returns%20the%20angle%20theta%20of,from%20%2DPI%20to%20PI%20radians.
            shooter.setAngle(angle.coerceIn(0.0, 0.887))
        }
        if(shoot() && shooter.noteIn()) {
            when (shootingState) {
                ShootingState.Idle -> {
                    shootingState = ShootingState.Revving
                    shooter.startShooting(false)
                    //SmartDashboard.putBoolean("revving", false);
                    shooter.coastMode()
                }
                ShootingState.Revving -> {
                    if (shooter.atSpeed(false)) {
                        shootingState = ShootingState.Shooting
                        shooter.intake()
                    }
                }
                ShootingState.Shooting -> {
                    if(!shooter.noteIn()) {
                        shootingState = ShootingState.Idle
                        shooter.stopShooting()
                        shooter.stopIntaking()
                    }
                }
            }
        } else {
            shootingState = ShootingState.Idle
            shooter.stopShooting()
            shooter.stopIntaking()
        }
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return shooter.atAngle() && terminate;
    }

    override fun end(interrupted: Boolean) {
        SmartDashboard.putBoolean("aim shooter running", false);
    }
}
