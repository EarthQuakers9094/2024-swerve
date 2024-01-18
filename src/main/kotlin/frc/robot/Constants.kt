package frc.robot

import com.pathplanner.lib.util.PIDConstants

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. inside the companion object). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
object Constants {
    object Drivebase {
        const val WHEEL_LOCK_TIME = 10.0
        const val MAX_AUTO_SPEEDS = 3.0
        const val RADIUS = 0.5

        const val MAX_ACCEL = 1.0
        const val MAX_ANGULAR_ACCELERATION = 0.5
        const val MAX_TURNING_SPEEDS = 3.0

        val TRANSLATION_PID = PIDConstants(0.2, 0.0, 0.0)
        val ROTATION_PID = PIDConstants(0.2, 0.0, 0.0)
    }
    object OperatorConstants {
        const val kDriverControllerPort = 0
        const val LEFT_X_DEADBAND = 0.01
        const val LEFT_Y_DEADBAND = 0.01
    }
    object Intake {
        const val speed = 1.0
    }
}