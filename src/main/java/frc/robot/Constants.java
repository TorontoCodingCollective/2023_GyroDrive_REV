// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose.
 * <p>
 * All constants should be declared globally (i.e. public static).
 * <br>
 * Do not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double DEFAULT_COMMAND_TIMEOUT_SECONDS = 5;

    public static final class OperatorInputConstants {

        public static final int DRIVER_CONTROLLER_PORT   = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static final class AutoConstants {

        public static enum AutoPattern {
            DO_NOTHING, DRIVE_FORWARD
        }
    }



    public static final class DriveConstants {

        public static enum DriveMode {
            TANK, ARCADE, SINGLE_STICK_LEFT, SINGLE_STICK_RIGHT;
        }

        public static final int     LEFT_MOTOR_PORT      = 10;
        public static final int     RIGHT_MOTOR_PORT     = 20;

        public static final boolean LEFT_MOTOR_REVERSED  = false;
        public static final boolean RIGHT_MOTOR_REVERSED = true;

        public static final double  CM_PER_ENCODER_COUNT = 3.503;

        public static final boolean GYRO_REVERSED        = false;

        /** Proportional gain for gyro pid tracking */
        public static final double  GYRO_PID_KP          = 0;

        public static final double  DRIVE_SCALING_BOOST  = 1;
        public static final double  DRIVE_SCALING_NORMAL = .6;
        public static final double  DRIVE_SCALING_SLOW   = .3;
    }

}
