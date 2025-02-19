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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.ironwolves.ftc.navutils.AutonomousNavigator;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that test in either the autonomous or the teleop period of an FTC match.
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

@Autonomous(name = "OldApril", group = "Linear Opmode")
public class OldApril extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();

    Orientation angles;

    int targetLiftPostion = 10;
    int lastTargetLiftPostion = targetLiftPostion;
    int targetLiftPostionIndex = 0;

    boolean targetClawOpen = false;
    double targetClawPosition = 0.45f;

    PositionCalculator posCalc;
    AutonomousNavigator autoNav;

    int test = 0;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, Config.Hardware.Motor.frontLeftMotorName);
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, Config.Hardware.Motor.frontRightMotorName);
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, Config.Hardware.Motor.backLeftMotorName);
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, Config.Hardware.Motor.backRightMotorName);

        posCalc = new PositionCalculator(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
        autoNav = new AutonomousNavigator(posCalc);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam cvWebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);


        DcMotorEx winchMotor = hardwareMap.get(DcMotorEx.class, Config.Hardware.Motor.winchMotorName);

        Servo clawServo = hardwareMap.servo.get(Config.Hardware.Servo.clawOpenServoName);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");


        frontLeftMotor.setDirection(Config.Hardware.Motor.frontLeftMotorDirection);
        frontRightMotor.setDirection(Config.Hardware.Motor.frontRightMotorDirection);
        backLeftMotor.setDirection(Config.Hardware.Motor.backLeftMotorDirection);
        backRightMotor.setDirection(Config.Hardware.Motor.backRightMotorDirection);

        clawServo.setDirection(Config.Hardware.Servo.clawServoDirection);

        winchMotor.setDirection(Config.Hardware.Motor.winchMotorDirection);
        clawServo.setPosition(Config.Hardware.Servo.clawOpenPostion);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setTargetPosition(0);
        frontRightMotor.setTargetPosition(0);
        backLeftMotor.setTargetPosition(0);
        backRightMotor.setTargetPosition(0);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        winchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        winchMotor.setTargetPosition(0); //must set target postion before enableing RUN_TO_POSITION
        winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(.042, 1430,1430,480,620);//tagsize in meters; last 4 numbers can be found here: https://horus.readthedocs.io/en/release-0.2/source/scanner-components/camera.html
        cvWebcam.setPipeline(aprilTagDetectionPipeline);
        cvWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                cvWebcam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                telemetry.addLine("Camera open error!");
            }
        });

        ArrayList<Integer> detectedTags = new ArrayList<>();
        while (!isStarted() && !isStopRequested()){
            detectedTags.clear();
            for (AprilTagDetection detection :
                    aprilTagDetectionPipeline.getLatestDetections()) {
                detectedTags.add(detection.id);
                telemetry.addData("Detected Tag # ", detection.id);
            }
            if (aprilTagDetectionPipeline.getLatestDetections().isEmpty()){
                telemetry.addLine("No Tags Detected");
            }
            telemetry.update();
        }
        runtime.reset();

        autoNav.move(new Position(DistanceUnit.METER, 0, .7, 0, 500), frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
        if (detectedTags.isEmpty() || detectedTags.contains(Config.Software.AprilTags.ZONE_2_ID)){
            //do nothing
        } else if (detectedTags.contains(Config.Software.AprilTags.ZONE_1_ID)){
            autoNav.move(new Position(DistanceUnit.METER, -.6, 0, 0, 500), frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
        } else if (detectedTags.contains(Config.Software.AprilTags.ZONE_3_ID)){
            autoNav.move(new Position(DistanceUnit.METER, .6, 0, 0, 500), frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            autoNav.run();
            telemetry.update();
        }

    }




}
