import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.robot.commands.CommandSequence
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Shooter

class Shoot(
        private val shooter: Shooter,
        private val elevator: Elevator,
        private val shootSpeed: Shooter.ShootSpeed
) : CommandSequence() {

    val supplier = { shooter.atSpeed(shootSpeed) }

    val inShooter = { !shooter.noteIn() }

    override val commands: List<Command> =
            listOf(
                    InstantCommand(
                            object : Runnable {
                                override fun run() {
                                    shooter.startShooting(shootSpeed)
                                    SmartDashboard.putBoolean("intaking shooter on shoot", false)
                                    shooter.coastMode()
                                }
                            },
                            shooter
                    ),
                    WaitUntilCommand(supplier),
                    InstantCommand(
                            object : Runnable {
                                override fun run() {
                                    shooter.intake()
                                    SmartDashboard.putBoolean("intaking shooter on shoot", true)
                                }
                            },
                            shooter
                    ),
                    WaitCommand(0.5),
                    WaitUntilCommand(inShooter),
                    WaitCommand(0.5)
            )

    override fun finally(interrupted: Boolean) {
        shooter.stopIntaking()
        shooter.stopShooting()
        SmartDashboard.putBoolean("intaking shooter on shoot", false)

        // GotoPose(shooter, elevator, Pose(0.0, 0.0), false).schedule();
    }
}
//
//
