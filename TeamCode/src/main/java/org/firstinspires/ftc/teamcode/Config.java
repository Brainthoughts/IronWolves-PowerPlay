package org.firstinspires.ftc.teamcode;

public class Config {
    public static class Hardware {
        public static class Motor {
            public static String frontLeftMotorName = "leftFront";
            public static String frontRightMotorName = "rightFront";
            public static String backLeftMotorName = "leftRear";
            public static String backRightMotorName = "rightRear";
            public static String winchMotorName = "winch_motor";

            public static double driveMotorPPR =  ((1+(46.0/11)) * 28);
            public static double winchMotorPPR = ((((1+(46d/17))) * (1+(46d/11))) * 28);

        }

        public static class Servo {
            public static String clawServoName = "claw_open_servo";
            static double clawOpenPostion = 0.36;
            static double clawClosedPosition = 0.47;
        }

        public static class Wheel {
            public static double wheelRadius = 0.048;
            static double gearRatio = 10d/14d;
            static double distanceFromCenter = 0.1703;
            //42.6 degrees for 1 wheel rotation
            //1111 ticks per full rotation of robot
            //4.5 in to center on y
            //5 in to center x
            //0.048/(0.1703)*360*.42
        }
    }

    public static class Software{
        public static class AprilTags{
            static int ZONE_1_ID = 213;
            static int ZONE_2_ID = 214;
            static int ZONE_3_ID = 215;
        }
    }
}
