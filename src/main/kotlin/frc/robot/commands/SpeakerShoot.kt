import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants
import frc.robot.commands.CommandSequence
import frc.robot.commands.GotoPose
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Intake

class SpeakerShoot(private val elevator: Elevator, private val shooter: Shooter, private val intake: Intake) :
        CommandSequence() {

    override val commands: List<Command> =
            listOf(
                InstantCommand(
                        object : Runnable {
                            override fun run() {
                                shooter.disableUpdates = true
                                shooter.startShooting(false);
                                SmartDashboard.putBoolean("in speaker shoot", true)
                            }
                        },
                        shooter
                ),
                GotoPose(shooter, elevator, Constants.Poses.speakerShoot, false),
                InstantCommand(object: Runnable{override fun run() {intake.startIntaking()}}, intake),
                Shoot(shooter,elevator,Shooter.ShootSpeed.Speaker).build(),
                // InstantCommand(object: Runnable{override fun run() {intake.stopIntaking()}}, intake),
                
            )

    override fun finally(interrupted: Boolean) {
        shooter.setAngle(0.0)
        shooter.disableUpdates = false
        SmartDashboard.putBoolean("in speaker shoot", false)
        intake.stopIntaking()
    }
    
}
//
//
