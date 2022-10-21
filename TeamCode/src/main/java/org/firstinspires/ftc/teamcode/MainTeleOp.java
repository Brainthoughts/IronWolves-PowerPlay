/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 * <p>
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 * <p>
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 * <p>
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 * <p>
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 * <p>
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "MainTeleOp", group = "Linear Opmode")
public class MainTeleOp extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx frontLeftMotor = null;
    private DcMotorEx frontRightMotor = null;
    private DcMotorEx backLeftMotor = null;
    private DcMotorEx backRightMotor = null;
    private DcMotorEx winchMotor = null;

    private Servo clawOpenServo = null;

    int targetLiftPostion = 0;
    int lastTargetLiftPostion = targetLiftPostion;
    int targetLiftPostionIndex = 0;

    boolean targetClawOpen = false;

    private final Gamepad previousGamepad1 = new Gamepad();

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        try {
            previousGamepad1.fromByteArray(gamepad1.toByteArray());
        } catch (RobotCoreException e) {
            e.printStackTrace();
        }
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class,"front_right_motor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class,"back_left_motor");
        backRightMotor = hardwareMap.get(DcMotorEx.class,"back_right_motor");

        winchMotor = hardwareMap.get(DcMotorEx.class, "winch_motor");

        clawOpenServo = hardwareMap.servo.get("claw_open_servo");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        clawOpenServo.setDirection(Servo.Direction.FORWARD);

        winchMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        winchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        winchMotor.setTargetPosition(0); //must set target postion before enableing RUN_TO_POSITION
        winchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            drive();
            lift();
            claw();
            telemetry.update();

            try {
                previousGamepad1.fromByteArray(gamepad1.toByteArray());
            } catch (RobotCoreException e) {
                e.printStackTrace();
            }
        }
    }

    void drive() {
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double vertical = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double horizontal = gamepad1.left_stick_x;
        double rotation = gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double frontLeftPower = vertical + horizontal + rotation;
        double frontRightPower = vertical - horizontal - rotation;
        double backLeftPower = vertical - horizontal + rotation;
        double backRightPower = vertical + horizontal - rotation;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send calculated power to wheels
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
        telemetry.addData("Left Joystick 1", "%4.2f, %4.2f", gamepad1.left_stick_x, gamepad1.left_stick_y);
        telemetry.addData("Left Joystick 2", "%4.2f, %4.2f", gamepad2.left_stick_x, gamepad2.left_stick_y);
        telemetry.addData("Vertical", "%4.2f", vertical);
        telemetry.addData("Horizontal", "%4.2f", horizontal);
        telemetry.addData("Rotation", "%4.2f", rotation);

    }

    void lift() {
        int liftMin = 10;
        int liftMax = 1830;
        int liftSpeed = 500;
        int[] postions = {0, 300, 600, 900, 1200, 1500, 1800};

        if (targetLiftPostionIndex > 0 && previousGamepad1.left_bumper != gamepad1.left_bumper) {
            targetLiftPostionIndex -= gamepad1.left_bumper ? 1 : 0; //if the button was pressed down, lower the targetLiftPosition
            targetLiftPostion = postions[targetLiftPostionIndex];
        } else if (targetLiftPostionIndex < postions.length - 1 && previousGamepad1.right_bumper != gamepad1.right_bumper) {
            targetLiftPostionIndex += gamepad1.right_bumper ? 1 : 0; //if the button was pressed down, raise the targetLiftPosition
            targetLiftPostion = postions[targetLiftPostionIndex];
        }


        if (gamepad1.right_trigger - gamepad1.left_trigger != 0) {
            if (DcMotor.RunMode.RUN_USING_ENCODER != winchMotor.getMode())
                winchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (winchMotor.getCurrentPosition() < liftMin) {
                winchMotor.setPower(gamepad1.right_trigger);
            } else if (winchMotor.getCurrentPosition() > liftMax) {
                winchMotor.setPower(gamepad1.left_trigger);
            } else {
                winchMotor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            }
            targetLiftPostion = Math.min(Math.max(winchMotor.getCurrentPosition(), liftMin), liftMax); //keep target position in range
        } else if (DcMotor.RunMode.RUN_TO_POSITION != winchMotor.getMode()) {
            winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (lastTargetLiftPostion != targetLiftPostion) {
            winchMotor.setTargetPosition(targetLiftPostion);
            winchMotor.setVelocity(targetLiftPostion > lastTargetLiftPostion ? liftMax : -liftSpeed);
        }

//        telemetry.addData("LiftTarget", "%4.2f", targetLiftPostion);
//        telemetry.addData("TargetPostion", "%4.2f", postions[targetLiftPostion]);
        telemetry.addData("CurrentPosition", "%d", winchMotor.getCurrentPosition());
        telemetry.addData("TargetPosition", "%d", targetLiftPostion);
        telemetry.addData("Left Trigger", "%f", gamepad1.left_trigger);
        telemetry.addData("Right Trigger", "%f", gamepad1.right_trigger);
        telemetry.addData("Mode", "%s", winchMotor.getMode());

    }

    void claw() {
        double openPostion = 0.5;

        if (previousGamepad1.a != gamepad1.a && gamepad1.a) {
            targetClawOpen = !targetClawOpen;
            clawOpenServo.setPosition(targetClawOpen ? 0 : openPostion); //if the button was pressed down, toggle the claw


        }
    }
}
