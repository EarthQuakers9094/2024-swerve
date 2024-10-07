import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.robot.commands.AimShooter
import frc.robot.commands.CommandSequence
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Swerve
import org.photonvision.PhotonCamera

class ShootTime(
        private val shooter: Shooter,
        private val intake: Intake,
        private val elevator: Elevator,
        private val swerveDrive: Swerve,
        private val camera: PhotonCamera
) : CommandSequence() {

    val supplier = { shooter.atSpeed(false) }

    override val commands: List<Command> =
            listOf(
                    ParallelDeadlineGroup(
                            WaitUntilCommand({
                                camera.latestResult.targets
                                        .filter { it.fiducialId == 4 || it.fiducialId == 7 }
                                        .size == 0
                            }),
                            SequentialCommandGroup(
                                    InstantCommand(
                                            object : Runnable {
                                                override fun run() {
                                                    intake.startIntaking()
                                                    shooter.startShooting(false)
                                                }
                                            },
                                            intake
                                    ),
                                    AlignSpeaker(camera, swerveDrive),
                                    // FaceDirection(swerveDrive, { swerveDrive.speakerAngle() },
                                    // false)
                                    /*.alongWith( */ AimShooter(
                                            shooter,
                                            swerveDrive,
                                            true,
                                            camera,
                                            { false }
                                    ) /*)*/,
                                    WaitCommand(0.25),
                                    Shoot(shooter, elevator, Shooter.ShootSpeed.Speaker).build(),
                            ),
                    )
            )

    override fun finally(interrupted: Boolean) {
        shooter.stopIntaking()
        intake.stopIntaking()
        shooter.stopShooting()
    }
}
//
//
