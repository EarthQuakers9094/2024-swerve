import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.robot.Constants
import frc.robot.commands.CommandSequence
import frc.robot.commands.GotoPose
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter
import java.util.function.BooleanSupplier

class Pickup(
        private val shooter: Shooter,
        private val elevator: Elevator,
        private val intake: Intake,
        private val high: Boolean
) : CommandSequence() {

    val supplier: BooleanSupplier = BooleanSupplier { shooter.noteIn() }

    override val commands: List<Command> =
            listOf(
                    GotoPose(
                            shooter,
                            elevator,
                            if (high) {
                                Constants.Poses.highPickup
                            } else {
                                Constants.Poses.pickup
                            },
                            false
                    ),
                    InstantCommand(
                            object : Runnable {
                                override fun run() {
                                    SmartDashboard.putNumber("running pickup", 1.0)
                                    if (high) {
                                        shooter.back()
                                    } else {
                                        shooter.intake()
                                        intake.startIntaking()
                                    }
                                }
                            },
                            shooter,
                            intake
                    ),
                    WaitUntilCommand(supplier),
                    InstantCommand(
                            object : Runnable {
                                override fun run() {
                                    intake.stopIntaking()
                                    shooter.stopIntaking()
                                    SmartDashboard.putNumber("running pickup", 2.0)
                                }
                            },
                            intake,
                            shooter
                    )
            )

    override fun finally(interrupted: Boolean) {
        SmartDashboard.putNumber("running pickup", 3.0)
        intake.stopIntaking()
        shooter.stopShooting()
        shooter.stopIntaking()
        shooter.setAngle(0.0)
        elevator.setPosition(0.0)
    }
}
