package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Config {
    public static class Hardware {
        public static class Motor {
            public static String frontLeftMotorName = "front_left_motor";
            public static String frontRightMotorName = "front_right_motor";
            public static String backLeftMotorName = "back_left_motor";
            public static String backRightMotorName = "back_right_motor";
            public static String winchMotorName = "winch_motor";

            public static double driveMotorPPM =  ((1+(46.0/11)) * 28);
            public static double winchMotorPPM = ((((1+(46d/17))) * (1+(46d/11))) * 28);

        }

        public static class Servo {
            public static String clawServo = "claw_open_servo";
            static double clawOpenPostion = 0.36;
            static double clawClosedPosition = 0.47;
        }

        public static class Wheel {
            public static double wheelRadius = 0.048;
            double gearRatio = 10d/14d;
            double distanceFromCenter = 0.1703;
            //42.6 degrees for 1 wheel rotation
            //1111 ticks per full rotation of robot
            //4.5 in to center on y
            //5 in to center x
            //0.048/(0.1703)*360*.42
        }
    }
}
