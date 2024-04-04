package frc.robot.commands

import com.pathplanner.lib.util.PIDConstants
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Swerve
import frc.robot.utils.MovingAverage
import frc.robot.commands.CollectNote
import org.photonvision.PhotonCamera
import java.util.function.BooleanSupplier

class CollectNote(
        private val rotationPidConstants: PIDConstants,
        //private val frontCamera: PhotonCamera,
        private val backCamera: PhotonCamera,
        private val swerve: Swerve,
        private val dataInconsistency: Int,
        private val hasNote: BooleanSupplier,
        private val teleop: Boolean,
        // private val intake: Intake?
) : Command() {
    private var forward = true
    private val targetYaw = MovingAverage(dataInconsistency)
    private var updatesSinceLastTarget = 0

    private val rotationPID =
            PIDController(rotationPidConstants.kP, rotationPidConstants.kI, rotationPidConstants.kD)

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        // if (intake != null) {
        //     addRequirements(intake, swerve)
        // } else {
            addRequirements(swerve)
        //}
    }

    /** The initial subroutine of a command. Called once when the command is initially scheduled. */
    override fun initialize() {
        updatesSinceLastTarget = 0
        for (unused in 1..10) {
            targetYaw.addValue(0.0)
        }
        forward = false//CollectNote.pickDirection(frontCamera, backCamera)
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled. (That is, it is
     * called repeatedly until [isFinished] returns true.)
     */
    override fun execute() {
        SmartDashboard.putData("notepid", rotationPID)
        SmartDashboard.putNumber("updates  since collect note had its last target", updatesSinceLastTarget.toDouble())
        
        val camera = backCamera
        
        val res = camera.latestResult

        SmartDashboard.putNumber("number of targets", res.targets.size.toDouble())
        var calculation = 0.0
        var speedFactor = 1.0
        if(!teleop) {
        // if (updatesSinceLastTarget > (30)) {
        //     intake?.stopIntaking()
        // } else {
        //     intake?.startIntaking()
        // }
    }
        if (!hasNote.asBoolean && res.hasTargets() && res.bestTarget.area > 2.0) {
            updatesSinceLastTarget = 0
            val target = res.bestTarget
            val averageYaw = targetYaw.addValue(target.yaw)
            SmartDashboard.putNumber("note yaw", averageYaw)
            calculation = rotationPID.calculate(averageYaw)
            if (target.area > 15) {
                speedFactor = 0.5
            }
        } else {
            updatesSinceLastTarget++
        }
        if (updatesSinceLastTarget <= (0)) {
            val direction = if (forward) {1.0} else {-1.0}
            swerve.drive(Translation2d(2.0 * (direction) /*-0.75*/, 0.0), calculation, false)
        }
    }

    /**
     * Returns whether this command has finished. Once a command finishes -- indicated by this
     * method returning true -- the scheduler will call its [end] method.
     *
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use [InstantCommand][edu.wpi.first.wpilibj2.command.InstantCommand] for such
     * an operation.
     *
     * @return whether this command has finished.
     */
    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return updatesSinceLastTarget > (0)
    }

    /**
     * The action to take when the command ends. Called when either the command finishes normally --
     * that is, it is called when [isFinished] returns true -- or when it is interrupted/canceled.
     * This is where you may want to wrap up loose ends, like shutting off a motor that was being
     * used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    override fun end(interrupted: Boolean) {}


    companion object {
        fun pickDirection(frontCamera: PhotonCamera, backCamera: PhotonCamera): Boolean {
            return false
            val frontResult = frontCamera.latestResult
            val backResult = backCamera.latestResult
           if(frontResult.hasTargets() && backResult.hasTargets()) {
                return frontResult.bestTarget.area > backResult.bestTarget.area
            } else if (frontResult.hasTargets()) {
                return true
            } else {
                return false
            }
        }
    }
}
