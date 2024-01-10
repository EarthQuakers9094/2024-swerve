package frc.robot

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. inside the companion object). Do not put anything functional in this class.
 *
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
class Constants {
    object Drivebase {
        const val WHEEL_LOCK_TIME = 10.0;
        const val MAX_AUTO_SPEEDS = 3.0;
        const val RADIUS = 0.5;
    }
    object OperatorConstants {
        const val kDriverControllerPort = 0
        const val LEFT_X_DEADBAND = 0.01;
        const val LEFT_Y_DEADBAND = 0.01;
    }
}
