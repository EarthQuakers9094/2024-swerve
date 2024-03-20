import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.Constants
import frc.robot.commands.CommandSequence
import frc.robot.commands.GotoPosePar
import frc.robot.subsystems.Elevator
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.subsystems.Shooter
import org.photonvision.PhotonCamera

class LocationShoot(
        private val elevator: Elevator, 
        private val shooter: Shooter,
        private val pose:Pose,
        private val elevatorFirst: Boolean,
        ) :
        CommandSequence() {

    override val commands: List<Command> =
            listOf(
                InstantCommand(
                        object : Runnable {
                            override fun run() {
                                shooter.startShooting(false);
                            }
                        },
                ),
                GotoPosePar(shooter, elevator, pose),
                
                Shoot(shooter,elevator,false).build(),
                WaitCommand(1.0)
            )

    override fun finally(interrupted: Boolean) {
        shooter.setAngle(0.0)
    }
}
//
//
