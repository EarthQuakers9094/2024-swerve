import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.Swerve
import org.photonvision.PhotonCamera

class AlignSpeaker(val camera: PhotonCamera, val swerve: Swerve) : Command() {
    private val rotationPID =
            PIDController(
                    Constants.Camera.rotationPid.kP,
                    Constants.Camera.rotationPid.kI,
                    Constants.Camera.rotationPid.kD
            )
    private var lastResult = camera.latestResult
    init {
        addRequirements(swerve)
    }

    override fun execute() {
        lastResult = camera.latestResult

        val targetyaw =
                -1 *
                        (lastResult
                                .targets
                                .filter { it.fiducialId == 4 || it.fiducialId == 7 }
                                .getOrNull(0)
                                ?.yaw
                                ?: 0.0)
        val angVelocity = rotationPID.calculate(targetyaw)

        swerve.drive(
                Translation2d(0.0, Rotation2d(0.0)),
                -angVelocity * Constants.Drivebase.MAX_TURNING_SPEEDS,
                false
        )
    }

    override fun isFinished(): Boolean {
        return (lastResult
                .targets
                .filter { it.fiducialId == 4 || it.fiducialId == 7 }
                .getOrNull(0)
                ?.yaw
                ?: 0.0) < 0.1
    }
}
