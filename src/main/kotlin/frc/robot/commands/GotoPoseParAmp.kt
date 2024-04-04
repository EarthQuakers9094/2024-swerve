package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Elevator
import org.photonvision.PhotonCamera
import kotlin.math.atan2
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import Pose

class GotoPoseParAmp(private val shooter: Shooter,private val elevator: Elevator) : SequentialCommandGroup() {
    init {
        // each subsystem used by the command must be passed into the addRequiresments() method
        val height = SetValue.setHeight(elevator,  Constants.Poses.amp.height);
        val angle = SetValue.setShootingAngle(shooter, Constants.Poses.amp.angle);
        

        addCommands(height.alongWith(WaitCommand(0.8).andThen(angle)));
    }
}
