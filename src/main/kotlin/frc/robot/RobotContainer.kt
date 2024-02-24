package frc.robot

import FollowTrajectory
import Pickup
import ShootTime
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.util.PIDConstants
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.robot.commands.Brake
import frc.robot.commands.CollectNote
import frc.robot.commands.FaceDirection
import frc.robot.commands.SetValue
import frc.robot.commands.swervedrive.drivebase.TeleopDrive
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Swerve
import frc.robot.utils.Config
import org.photonvision.PhotonCamera

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
    // The robot's subsystems and commands are defined here...

    private val aprilCamera = PhotonCamera("BW")
    private val noteCamera = PhotonCamera("CLR")
    private val swerveDrive = Swerve(/*aprilCamera*/ )

    private var elevator: Elevator? = null
    // (Constants.Elevator.motorID)
    // (Constants.Elevator.motorID)

    private var shooter: Shooter? = null

    //         Shooter(
    //                 Constants.Shooter.topCanid,
    //                 Constants.Shooter.bottomCanID,
    //                 Constants.Shooter.shooterJointCanID,
    //                 Constants.Shooter.shooterJoint2CanID
    //         )

    private var intake: Intake? = null

    val operatorExtra = XboxController(Constants.OperatorConstants.kDriverControllerPort)
    val driverLeftStick = Joystick(Constants.OperatorConstants.driverLeftStickPort)
    val driverRightStick = Joystick(Constants.OperatorConstants.driverRightStickPort)

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    init {
        // Configure the trigger bindings

        val onTest = Config(true, false)

        if (!onTest.config) {
            intake =
                    Intake(
                            Constants.Intake.motorid,
                            Constants.Intake.followMotorId,
                            Constants.Intake.frontIntakeId
                    )
            shooter =
                    Shooter(
                            Constants.Shooter.topCanid,
                            Constants.Shooter.bottomCanID,
                            Constants.Shooter.shooterJointCanID,
                            Constants.Shooter.intakeMotorID
                    )
            elevator = Elevator(Constants.Elevator.motorID, Constants.Elevator.followMotorID)

            NamedCommands.registerCommand("pickup", Pickup(shooter!!, elevator!!, intake!!).build())
            NamedCommands.registerCommand(
                    "faceSpeaker",
                    FaceDirection(swerveDrive, { swerveDrive.speakerAngle() }, false)
            )
            NamedCommands.registerCommand(
                    "shoot",
                    ShootTime(shooter!!, intake!!, elevator!!, swerveDrive, aprilCamera).build()
            )
            NamedCommands.registerCommand(
                    "facedown",
                    FaceDirection(swerveDrive, { Rotation2d.fromRadians(-Math.PI / 2.0) }, false)
            )
        }

        configureBindings()

        fun applyPov(direction: Int, speed: Double): Double {
            if (direction == -1) {
                return speed * 0.75
            } else if (direction == 0) {
                return speed
            } else if (direction == 180) {
                return speed * 0.5
            }

            return speed
        }

        val leftY = {
            MathUtil.applyDeadband(
                    applyPov(driverLeftStick.getPOV(), driverLeftStick.getY()),
                    Constants.OperatorConstants.LEFT_Y_DEADBAND
            )
            MathUtil.applyDeadband(
                    applyPov(driverLeftStick.getPOV(), driverLeftStick.getY()),
                    Constants.OperatorConstants.LEFT_Y_DEADBAND
            )
        }

        val leftX = {
            MathUtil.applyDeadband(
                    applyPov(driverLeftStick.getPOV(), driverLeftStick.getX()),
                    Constants.OperatorConstants.LEFT_X_DEADBAND
            )
            MathUtil.applyDeadband(
                    applyPov(driverLeftStick.getPOV(), driverLeftStick.getX()),
                    Constants.OperatorConstants.LEFT_X_DEADBAND
            )
        }

        val omega = {
            MathUtil.applyDeadband(
                    driverRightStick.getX(),
                    driverRightStick.getX(),
                    Constants.OperatorConstants.LEFT_X_DEADBAND
            )
        }

        val driveMode = { true }

        val faceSpeaker = { driverRightStick.getRawButton(7) }

        val simClosedFieldRel =
                TeleopDrive(swerveDrive, leftY, leftX, omega, driveMode, faceSpeaker)

        swerveDrive.defaultCommand = simClosedFieldRel
    }

    fun debugPeriodic() {
        SmartDashboard.putBoolean("Connected or nay", aprilCamera.isConnected)
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * [Trigger#Trigger(java.util.function.BooleanSupplier)] constructor with an arbitrary
     * predicate, or via the named factories in
     * [edu.wpi.first.wpilibj2.command.button.CommandGenericHID]'s subclasses for
     * [CommandXboxController]/[edu.wpi.first.wpilibj2.command.button.CommandPS4Controller]
     * controllers or [edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureBindings() {
        // Schedule ExampleCommand when exampleCondition changes to true
        // Trigger { exampleSubsystem.exampleCondition() }.onTrue(ExampleCommand(exampleSubsystem))

        // Schedule exampleMethodCommand when the Xbox controller's B button is pressed,
        // cancelling on release.
        // driverController.b().whileTrue(exampleSubsystem.exampleMethodCommand())
        if (shooter != null && intake != null && elevator != null) {
            SmartDashboard.putBoolean("shooter", true)

            JoystickButton(driverLeftStick, 2)
                    .onTrue(
                            ShootTime(shooter!!, intake!!, elevator!!, swerveDrive, aprilCamera)
                                    .build()
                    )

            JoystickButton(operatorExtra, 1).whileTrue(shooter!!.shootButton())
            JoystickButton(operatorExtra, 2).whileTrue(shooter!!.backButton())

            JoystickButton(operatorExtra, 3).whileTrue(intake!!.intake())
            JoystickButton(operatorExtra, 4).whileTrue(intake!!.backButton())

            JoystickButton(operatorExtra, 7).whileTrue(elevator!!.up())
            JoystickButton(operatorExtra, 8).whileTrue(elevator!!.down())

            JoystickButton(operatorExtra, 9).onTrue(SetValue.setHeight(elevator!!, true, 47.0))
            JoystickButton(operatorExtra, 10).onTrue(SetValue.setHeight(elevator!!, true, 0.0))

            JoystickButton(driverLeftStick, 3)
                    .whileTrue(Pickup(shooter!!, elevator!!, intake!!).build())

            // JoystickButton(operatorExtra,
            // 7).whileTrue(FaceDirection(swerveDrive,{swerveDrive.speakerAngle()},true));

            JoystickButton(driverLeftStick, 4)
                    .whileTrue(SetValue.setShootingAngle(shooter!!, true, 0.0))
            JoystickButton(driverLeftStick, 5)
                    .whileTrue(SetValue.setShootingAngle(shooter!!, true, Math.PI / 3))
        }

        JoystickButton(driverLeftStick, 1).whileTrue(Brake(swerveDrive))
        JoystickButton(driverRightStick, 2)
                .onTrue(FollowTrajectory(swerveDrive, PathPlannerPath.fromPathFile("to amp"), true))
    }

    fun setMotorBrake(enabled: Boolean) {
        swerveDrive.setMotorBrake(enabled)
    }

    fun periodic() {
        operatorExtra.setRumble(
                GenericHID.RumbleType.kBothRumble,
                (if (noteCamera.latestResult.hasTargets()) {
                    1.0
                } else {
                    0.0
                })
        )
    }

    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */
    val autonomousCommand: Command
        get() {
            // An example command will be run in autonomous
            return CollectNote(PIDConstants(0.045, 0.0, 0.001), noteCamera, intake, swerveDrive, 10)
        }
}
