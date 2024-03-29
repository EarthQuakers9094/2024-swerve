package frc.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.RobotBase
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.Constants
import frc.robot.utils.MovingAverage
import frc.robot.utils.configureSparkMax
import edu.wpi.first.wpilibj.DigitalInput
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.math.trajectory.TrapezoidProfile


/** Creates a new ExampleSubsystem. */
class Elevator(private val liftMotorId: Int, private val followMotorID: Int) : SubsystemBase() {
               private val liftSparkMax = CANSparkMax(liftMotorId, CANSparkLowLevel.MotorType.kBrushless);
               private val followMotor = CANSparkMax(followMotorID, CANSparkLowLevel.MotorType.kBrushless);


    private var elevator =
            ElevatorSim(
                    DCMotor.getNEO(2),
                    Constants.Elevator.gearing,
                    10.0,
                    0.2,
                    0.0,
                    0.762,
                    true,
                    0.762
            )

    private var sim_pid =
            PIDController(
                    Constants.Elevator.sim_pid.kP,
                    Constants.Elevator.sim_pid.kI,
                    Constants.Elevator.sim_pid.kD
            )

    private var pid = PIDController(Constants.Elevator.pid.kP,Constants.Elevator.pid.kI,Constants.Elevator.pid.kD)

    private var desiredPosition = 0.0

    private var averagePostion = MovingAverage(10)

    private var pidMode = true

    private var profile = TrapezoidProfile(TrapezoidProfile.Constraints(3.0*(60/25)*960.0,3.0*(60/25)*960.0));

    private var currentState = TrapezoidProfile.State(0.0,0.0);

    init {
        liftSparkMax.restoreFactoryDefaults()
        followMotor.restoreFactoryDefaults()

        while (!liftSparkMax.inverted) {
            liftSparkMax.inverted = true; // 0.052404
        }


        configureSparkMax {followMotor.follow(liftSparkMax,true)};

        //old: 25 new:60
        //TODO: try flipping this around i dont really want to try and figure it out
        configureSparkMax {liftSparkMax.encoder.setPositionConversionFactor(25.0 / 60.0)};
        configureSparkMax {liftSparkMax.encoder.setPosition(0.0)};
        configureSparkMax {liftSparkMax.setSmartCurrentLimit(60, 60)};
        configureSparkMax {followMotor.setSmartCurrentLimit(60, 60)};

        configureSparkMax {liftSparkMax.pidController.setP(Constants.Elevator.pid.kP)};
        configureSparkMax {liftSparkMax.pidController.setI(Constants.Elevator.pid.kI)};
        configureSparkMax {liftSparkMax.pidController.setD(Constants.Elevator.pid.kD)};
        configureSparkMax {liftSparkMax.setIdleMode(IdleMode.kBrake)}
        configureSparkMax {followMotor.setIdleMode(IdleMode.kBrake)}



        SmartDashboard.putData("sim pid", sim_pid)
        SmartDashboard.putData("elevator pid", pid);
        SmartDashboard.putNumber("desired elevator location", 0.1)

        if (!RobotBase.isReal()) {
            liftSparkMax.encoder.position = elevator.positionMeters
        }

        averagePostion.setAverage(liftSparkMax.encoder.position)
        liftSparkMax.burnFlash()
        followMotor.burnFlash()
    }

    fun enable() {
        desiredPosition = liftSparkMax.encoder.position;
        currentState = TrapezoidProfile.State(desiredPosition,0.0);
    }

    fun setClimbing(climbing: Boolean) {
        if (climbing) {
            pid = PIDController(Constants.Elevator.pid.kP,Constants.Elevator.pid.kI,Constants.Elevator.pid.kD);
        } else {
            pid = PIDController(Constants.Elevator.pid.kP,Constants.Elevator.pid.kI,Constants.Elevator.pid.kD)
        }
    }

    /** This method will be called once per scheduler run */
    override fun periodic() {
        SmartDashboard.putNumber("elevator position", liftSparkMax.encoder.position)
        averagePostion.addValue(liftSparkMax.encoder.position)
        SmartDashboard.putNumber("desired elevator position", desiredPosition)
        SmartDashboard.putNumber("lift spark max output current", liftSparkMax.outputCurrent)

        if (pidMode) {

            val nextPosition = profile.calculate(
                0.02, 
                currentState, 
                TrapezoidProfile.State(desiredPosition,0.0));

            // val output = pid.calculate(liftSparkMax.encoder.position, nextPosition.position) + Constants.Elevator.feedforward

            configureSparkMax {liftSparkMax.pidController.setReference(nextPosition.position, 
                CANSparkBase.ControlType.kPosition
            )}


            currentState = nextPosition;

            // SmartDashboard.putNumber("elevator output", output); 

            SmartDashboard.putNumber("elevator trapazoid position", nextPosition.position);
            SmartDashboard.putNumber("elevator current", liftSparkMax.outputCurrent)
            SmartDashboard.putNumber("elevator follower current", followMotor.outputCurrent)




            // liftSparkMax.set(output)
        }
    }


    fun setPosition(position: Double):Boolean {
        if (position <= Constants.Elevator.maxHeight && position >= 0) {
            desiredPosition = position
            pidMode = true
            return true;
        }

        DriverStation.reportError("elevator position set out of bounds",false)
        return false
    }

    fun up(): Command {
        var parent = this;
        return Commands.startEnd(object: Runnable {
                override fun run() {
                    liftSparkMax.set(Constants.Elevator.feedforward + 0.1);
                }
            },object: Runnable {
                override fun run() {
                    liftSparkMax.set(Constants.Elevator.feedforward);
                }
            },parent);
    }

    
    fun down(): Command {
        var parent = this;
        return Commands.startEnd(object: Runnable {
                override fun run() {
                    liftSparkMax.set(Constants.Elevator.feedforward - 0.1);
                }
            },object: Runnable {
                override fun run() {
                    liftSparkMax.set(Constants.Elevator.feedforward);
                }
            },parent);
    }

    fun atPosition(): Boolean {
        return abs(averagePostion.getAverage() - desiredPosition) <= 0.5
    }

    override fun simulationPeriodic() {
        val res =
                min(1.0, max(sim_pid.calculate(elevator.positionMeters, desiredPosition), -1.0)) +
                        Constants.Elevator.sim_feedforward
        SmartDashboard.putNumber("pid output", res)

        elevator.setInputVoltage(res * RobotController.getInputVoltage())

        elevator.update(0.02)

        liftSparkMax.encoder.position = elevator.positionMeters
    }
}

// 
// 