package org.firstinspires.ftc.teamcode.helpers;

public class Constants {

    public static class Drive {
        public static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
        public static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
        public static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
        public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        public static final double     DRIVE_SPEED             = 0.6;
        public static final double     TURN_SPEED              = 0.5;
    }

    public static class Drivetrain {
        public static final String LEFT_DRIVE_FRONT = "left_drive0";
        public static final String RIGHT_DRIVE_FRONT = "right_drive1";
        public static final String LEFT_DRIVE_REAR = "left_drive2";
        public static final String RIGHT_DRIVE_REAR = "right_drive3";
    }

    public static class Arm {
        public static final String ARM0 = "arm0";
        public static final String ARM1 = "arm1";
    }

    public static class Stage {
        public static final String STAGE0 = "stage0";
        public static final String STAGE1 = "stage1";
    }

    public static class Sensors {
        public static final String ARM_RESET = "reset";
        public static final String DISTANCE_SENSOR = "sense";
    }
}
