package frc.robot.subsystems

import com.revrobotics.AbsoluteEncoder
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand
import frc.robot.Constants
import frc.robot.utils.MovingAverage
import frc.robot.utils.configureSparkMax
import java.util.function.BooleanSupplier
import java.lang.Math

class Shooter(
        private val shooterCanID: Int,
        private val secondaryShooterID: Int,
        private val shooterJointCanID: Int,
        private val intakeMotorID: Int,
) : SubsystemBase() {
    private val jointAbsoluteEncoder = DutyCycleEncoder(1)


    private var startingPosition = absoluteAngle(); //(jointAbsoluteEncoder.get() * Math.PI * 2)// + 1.44558;

    private var speed = 0.0

    private var desiredAngle = 0.8

    private var angleRollingAverage = MovingAverage(20)

    private val shooterSparkMax = CANSparkFlex(shooterCanID, CANSparkLowLevel.MotorType.kBrushless)
    private val followerSparkMax =
            CANSparkFlex(secondaryShooterID, CANSparkLowLevel.MotorType.kBrushless)

    private val jointMotor1 = CANSparkFlex(shooterJointCanID, CANSparkLowLevel.MotorType.kBrushless)

    private val intakingMotor = CANSparkFlex(intakeMotorID, CANSparkLowLevel.MotorType.kBrushless)

    private var currentSetPoint = 0.0
    private var currentAverage = MovingAverage(10)

    private var topWheels = FlywheelSim(DCMotor.getNeoVortex(1), 1.0, 0.00176)

    private var joint =
            SingleJointedArmSim(DCMotor.getNEO(2), 30.0, 0.2, 0.43, 0.0, Math.PI / 2.0, true, 0.0)

    private var sim_top_pid =
            PIDController(
                    Constants.Shooter.sim_pid.kP,
                    Constants.Shooter.sim_pid.kI,
                    Constants.Shooter.sim_pid.kD
            )

    private var sim_bottom_pid =
            PIDController(
                    Constants.Shooter.sim_pid.kP,
                    Constants.Shooter.sim_pid.kI,
                    Constants.Shooter.sim_pid.kD
            )

    private var sim_joint_pid =
            PIDController(
                    Constants.Shooter.sim_join_pid.kP,
                    Constants.Shooter.sim_join_pid.kI,
                    Constants.Shooter.sim_join_pid.kD
            )

    private var bottomWheels = FlywheelSim(DCMotor.getNEO(1), 1.0, 14.4)

    private var inSensor = DigitalInput(Constants.Shooter.inSensorID)

    private var currentSetSpeed = 0.0

    private var profile = TrapezoidProfile(TrapezoidProfile.Constraints(60.0, 72.0*0.9))

    private var currentState = TrapezoidProfile.State(startingPosition, 0.0)

    private var lastAbsoluteAngle = 0.0;

    private var pid = PIDController(Constants.Shooter.join_pid.kP, Constants.Shooter.join_pid.kI, Constants.Shooter.join_pid.kD);

    private var stillUpdates = 0; 

    var disableUpdates = false;

    private var updates = 0    

    init {
        
        shooterSparkMax.restoreFactoryDefaults()
        followerSparkMax.restoreFactoryDefaults()
        jointMotor1.restoreFactoryDefaults()
        intakingMotor.restoreFactoryDefaults()

        lastAbsoluteAngle = absoluteAngle();

        // configureSparkMax {jointMotor1.setInverted(true q) };
        

        configureSparkMax {jointMotor1.pidController.setP(Constants.Shooter.join_pid.kP)};
        configureSparkMax {jointMotor1.pidController.setI(Constants.Shooter.join_pid.kI)};
        configureSparkMax {jointMotor1.pidController.setD(Constants.Shooter.join_pid.kD)};

        configureSparkMax {jointMotor1.encoder.setPositionConversionFactor(Constants.Shooter.positionConversionFactor)};
        //joinAbsoluteEncoder.distancePerRotation = 

        configureSparkMax {jointMotor1.encoder.setPosition(startingPosition)} //Constants.Shooter.startAngle

        // followerSparkMax.follow(shooterSparkMax, true)

        configureSparkMax {shooterSparkMax.encoder.setVelocityConversionFactor(
                Constants.Shooter.velocityConversionFactor)}

        configureSparkMax {shooterSparkMax.setSmartCurrentLimit(40, 40, 10_000_000)}
        configureSparkMax {followerSparkMax.setSmartCurrentLimit(40, 40, 10_000_000)}
        configureSparkMax {intakingMotor.setSmartCurrentLimit(40, 40, 10_000_000)}

        configureSparkMax {shooterSparkMax.pidController.setP(Constants.Shooter.p)}
        configureSparkMax {shooterSparkMax.pidController.setI(Constants.Shooter.i)}
        configureSparkMax {shooterSparkMax.pidController.setD(Constants.Shooter.d)}

        configureSparkMax {followerSparkMax.pidController.setP(Constants.Shooter.p)}
        configureSparkMax {followerSparkMax.pidController.setI(Constants.Shooter.i)}
        configureSparkMax {followerSparkMax.pidController.setD(Constants.Shooter.d)}
        // configureSparkMax {shooterSparkMax.pidController.setPositionPIDWrappingEnabled(true)}
        // configureSparkMax {shooterSparkMax.pidController.setPositionPIDWrappingMinInput(-Math.PI)}
        // configureSparkMax {shooterSparkMax.pidController.setPositionPIDWrappingMaxInput(Math.PI)}

        configureSparkMax {shooterSparkMax.setIdleMode(IdleMode.kBrake)}
        configureSparkMax {followerSparkMax.setIdleMode(IdleMode.kBrake)}

        SmartDashboard.putNumber("current set speed launcher", 0.0)
        SmartDashboard.putNumber("current set joint location launcher", 0.0)
        SmartDashboard.putData("joint pid", sim_joint_pid)
        configureSparkMax {followerSparkMax.follow(shooterSparkMax)}
                // configureSparkMax {followerSparkMax.follow(shooterSparkMax)}

        jointMotor1.burnFlash()
        shooterSparkMax.burnFlash()
        followerSparkMax.burnFlash()
    }

    fun resetAngle() {
        jointMotor1.encoder.position = absoluteAngle();
    }

    override fun periodic() {
        updates++
        
        if (RobotBase.isReal()) {
            speed = shooterSparkMax.encoder.velocity
        }

        desiredAngle = SmartDashboard.getNumber("shooter desired angle",desiredAngle)

        val nextPosition =
                profile.calculate(0.02,
                    currentState, 
                    TrapezoidProfile.State(desiredAngle, 0.0));

        configureSparkMax {jointMotor1.pidController.setReference(
                nextPosition.position,
                CANSparkBase.ControlType.kPosition
        )}

        currentState = nextPosition

        angleRollingAverage.addValue(jointMotor1.encoder.position)

        val aangle = absoluteAngle();

        SmartDashboard.putNumber("angular velocity absolute encoder", Math.abs((aangle - lastAbsoluteAngle) / 0.02));

        if (Math.abs((aangle - lastAbsoluteAngle) / 0.02) <= 0.04  && !disableUpdates) {
            stillUpdates = stillUpdates + 1;
            if (stillUpdates > 10 && absoluteAngle() < 2.0 && absoluteAngle() > 0.0) {
                jointMotor1.encoder.position = absoluteAngle();
            }
        } else {
            stillUpdates = 0;
        }

        if (updates <= 20 && updates >= 5) {
            jointMotor1.encoder.position = absoluteAngle();
        }

        lastAbsoluteAngle = absoluteAngle();


        SmartDashboard.putNumber("shooter angle", jointMotor1.encoder.position)
        SmartDashboard.putNumber("shooter angle velocity", jointMotor1.encoder.velocity)
        SmartDashboard.putNumber("pid angle joint", nextPosition.position)
        SmartDashboard.putNumber("jointValueAbsolute", jointAbsoluteEncoder.get())
        SmartDashboard.putNumber("jointValue", -(jointAbsoluteEncoder.get() * Math.PI * 2) + 5.194946)
        SmartDashboard.putNumber("shooter desired angle", desiredAngle)
        SmartDashboard.putNumber("still updates", stillUpdates.toDouble())


        SmartDashboard.putNumber("current motor speed launcher", speed)
        SmartDashboard.putNumber("shooter speed", shooterSparkMax.encoder.velocity)

        SmartDashboard.putBoolean("beam break", noteIn())

        SmartDashboard.putNumber("feader current", intakingMotor.outputCurrent)

        // SmartDashboard.putNumber("Is joint motor inverted", if (jointMotor1.inverted) {1.0} else {-1.0})
        // if(jointMotor1.inverted) {
        //     jointMotor1.inverted = false
        // }
    }

    fun enable() {
        currentState = TrapezoidProfile.State(jointMotor1.encoder.position, 0.0);
        // while (!jointMotor1.inverted) {
        //     jointMotor1.inverted = true
        // }
    }


    // fun configureSparkMax(config: () -> REVLibError) {
    //     for (int i = 0; i < maximumRetries; i++) {
    //         if (config.get() == REVLibError.kOk) {
    //             return;
    //         }
    //     }
    //     DriverStation.reportWarning("Failure configuring motor " + motor.getDeviceId(), true);
    // }

    fun absoluteAngle():Double {
        return -(jointAbsoluteEncoder.get() * Math.PI * 2) + 5.194946
    }

    fun setSpeed(speed: Double) {
        currentSetPoint = speed
        configureSparkMax {shooterSparkMax.pidController.setReference(speed, CANSparkBase.ControlType.kSmartVelocity)}
    }

    fun setAngle(angle: Double) {
        //jointMotor1.pidController.setReference(angle, CANSparkBase.ControlType.kPosition)
        desiredAngle = angle
        SmartDashboard.putNumber("shooter desired angle", desiredAngle)
        DriverStation.reportError("hello :3 from shooter", true)
    }

    fun coastMode() {
        configureSparkMax {shooterSparkMax.setIdleMode(IdleMode.kCoast)}
        configureSparkMax {followerSparkMax.setIdleMode(IdleMode.kCoast)}
    }

    fun atAngle(): Boolean {
        // return true
        return Math.abs(angleRollingAverage.getAverage() - desiredAngle) <= 0.1
    }

    fun setIntakingSpeed(speed: Double) {
        intakingMotor.set(speed)
        
        // shooterSparkMax.set(0.25);
    }

    fun intake() {
        setIntakingSpeed(Constants.Shooter.intakeSpeed)
        configureSparkMax {shooterSparkMax.setIdleMode(IdleMode.kBrake)}
        configureSparkMax {followerSparkMax.setIdleMode(IdleMode.kBrake)}

    }

    fun stopIntaking() {
        setIntakingSpeed(0.0)
    }

    fun intakeButtonCommand(): Command {
        val parent = this
        return Commands.startEnd(
                object : Runnable {
                    override fun run() {
                        parent.intake()
                    }
                },
                object : Runnable {
                    override fun run() {
                        parent.stopIntaking()
                    }
                },
                this
        )
    }

    fun startShooting(amp: Boolean) {
        val speed =
                if (amp) {
                    Constants.Shooter.ampSpeed
                } else {
                    Constants.Shooter.speed
                }
        currentSetSpeed = speed

        shooterSparkMax.set(speed)
    }

    fun stopShooting() {
        shooterSparkMax.set(0.0)
        currentSetSpeed = 0.0
    }

    fun shootButton(): Command {
        var parent = this
        return Commands.startEnd(
                object : Runnable {
                    override fun run() {
                        parent.startShooting(false)
                        parent.intake()
                    }
                },
                object : Runnable {
                    override fun run() {
                        parent.stopShooting()
                        parent.stopIntaking()
                    }
                },
                parent
        )
    }

    fun back(): Unit {
        shooterSparkMax.set(1.0)
        currentSetSpeed = 0.1
        intakingMotor.set(-0.2)
    }

    fun back2(): Unit {
        shooterSparkMax.set(0.70)
        currentSetSpeed = 0.1
        intakingMotor.set(-0.3)
    }


    fun backButton(): Command {
        var parent = this
        return Commands.startEnd(
                object : Runnable {
                    override fun run() {
                        back()
                    }
                },
                object : Runnable {
                    override fun run() {
                        parent.stopShooting()
                        parent.stopIntaking()
                    }
                },
                parent
        )
    }

    fun forwardButton(): Command {
        var parent = this
        return Commands.startEnd(
                object : Runnable {
                    override fun run() {
                        intakingMotor.set(1.0)
                    }
                },
                object : Runnable {
                    override fun run() {
                        parent.stopShooting()
                        parent.stopIntaking()
                    }
                },
                parent
        )
    }

    fun shootTime(intake: Intake, amp: Boolean): Command {
        val parent = this

        val supplier = { this.atSpeed(amp) }

        val command =
                Commands.sequence(
                        InstantCommand(
                                object : Runnable {
                                    override fun run() {
                                        parent.startShooting(amp)
                                    }
                                }
                        ),
                        WaitUntilCommand(supplier),
                        InstantCommand(
                                object : Runnable {
                                    override fun run() {
                                        parent.intake()
                                        intake.startIntaking()
                                    }
                                }
                        ),
                        WaitCommand(Constants.Shooter.shootTime),
                        InstantCommand(
                                object : Runnable {
                                    override fun run() {
                                        parent.stopIntaking()
                                        parent.stopShooting()
                                        intake.stopIntaking()
                                    }
                                }
                        )
                )

        command.addRequirements(this)

        return command
    }

    fun pickup(): Command {
        val parent = this

        val supplier: BooleanSupplier = BooleanSupplier { this.noteIn() }

        val command =
                Commands.sequence(
                        InstantCommand(
                                object : Runnable {
                                    override fun run() {
                                        parent.intake()
                                    }
                                }
                        ),
                        WaitUntilCommand(supplier)
                )

        command.addRequirements(this)

        return command.andThen(
                InstantCommand(
                        object : Runnable {
                            override fun run() {
                                parent.stopIntaking()
                            }
                        },
                        this
                )
        )
    }

    override fun simulationPeriodic() {
        SmartDashboard.putNumber("currentSetSpeed", currentSetSpeed)
        topWheels.setInputVoltage(currentSetSpeed * RobotController.getInputVoltage())

        bottomWheels.setInputVoltage(-currentSetSpeed * RobotController.getInputVoltage())

        val voltage =
                sim_joint_pid.calculate(joint.angleRads, desiredAngle).coerceIn(-1.0, 1.0) *
                        RobotController.getInputVoltage()

        SmartDashboard.putNumber("angle of arm", joint.angleRads)

        SmartDashboard.putNumber("input to arm voltage", voltage)

        joint.setInputVoltage(voltage)

        topWheels.update(0.02)
        bottomWheels.update(0.02)
        joint.update(0.02)

        jointMotor1.encoder.position = joint.angleRads

        speed = topWheels.angularVelocityRPM
    }

    fun noteIn(): Boolean {
        // return intakingMotor.outputCurrent > 35.0
        return !inSensor.get()
    }

    fun atSpeed(amp: Boolean): Boolean {
        if (RobotBase.isReal()) {
            speed = shooterSparkMax.encoder.velocity
        }

        return if (amp) {
            speed <= -2700.0
        } else {
            speed <= -4500.0
        }
    }
}
