package org.ironwolves.ftc.navutils;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.PositionCalculator;

public class Instruction {
    public enum Code {
        Move,
        Rotate,
        Custom,
        Claw,
        Lift,
        Sleep
    }
    private final Code instructionCode;
    private final Object[] parameters;
    private boolean firstIteration = true;
    private boolean complete = false;
    private PositionCalculator posCalc;

    public Instruction(Code instructionCode, Object[] parameters) {
        this.instructionCode = instructionCode;
        this.parameters = parameters;
    }

    public boolean isComplete(){
        return complete;
    }

    public void execute(PositionCalculator posCalc){
        this.posCalc = posCalc;
        switch (instructionCode){
            case Move:
                move();
                break;
            case Rotate:
                rotate();
                break;
            case Lift:
                lift();
                break;
            case Claw:
                claw();
                break;
            case Custom:
                custom();
                break;
            case Sleep:
                sleep();
                break;
        }
        firstIteration = false;
    }


    private void custom() {
        Runnable code = (Runnable) parameters[0];
        code.run();
        complete = true;
    }

    private void sleep() {
        long sleepTime = (long) parameters[0];
        if (System.currentTimeMillis() - sleepTime > 0){
            complete = true;
        }
    }

    private void claw() {
        double targetPosition = (double) parameters[0];
        Servo clawServo = (Servo) parameters[1];

        if (firstIteration){
            clawServo.setPosition(targetPosition);
        }

        if (clawServo.getPosition() - targetPosition == 0){
            complete = true;
        }

    }

    private void lift() {

    }

    private void rotate() {

    }

    private void move(){
        Position offset = (Position) parameters[0];
        DcMotorEx frontLeftMotor = (DcMotorEx) parameters[1];
        DcMotorEx frontRightMotor = (DcMotorEx) parameters[2];
        DcMotorEx backLeftMotor = (DcMotorEx) parameters[3];
        DcMotorEx backRightMotor = (DcMotorEx) parameters[4];

        int buffer = 250;

        if (firstIteration){
            //calculate and set new position for wheel encoders

            int frontLeftEncoder = frontLeftMotor.getCurrentPosition();
            int frontRightEncoder = frontRightMotor.getCurrentPosition();
            int backLeftEncoder = backLeftMotor.getCurrentPosition();
            int backRightEncoder = backRightMotor.getCurrentPosition();

            frontLeftEncoder += posCalc.distanceToMotorRotation(offset.y) + posCalc.distanceToMotorRotation(offset.x);
            frontRightEncoder += posCalc.distanceToMotorRotation(offset.y) - posCalc.distanceToMotorRotation(offset.x);
            backLeftEncoder += posCalc.distanceToMotorRotation(offset.y) - posCalc.distanceToMotorRotation(offset.x);
            backRightEncoder += posCalc.distanceToMotorRotation(offset.y) + posCalc.distanceToMotorRotation(offset.x);

            frontLeftMotor.setTargetPosition(frontLeftEncoder);
            frontRightMotor.setTargetPosition(frontRightEncoder);
            backLeftMotor.setTargetPosition(backLeftEncoder);
            backRightMotor.setTargetPosition(backRightEncoder);

            if (Math.abs(frontLeftMotor.getTargetPosition() - frontLeftMotor.getCurrentPosition()) < buffer){
                frontLeftMotor.setVelocity(buffer);
                frontRightMotor.setVelocity(buffer);
                backLeftMotor.setVelocity(buffer);
                backRightMotor.setVelocity(buffer);
            } else {
                frontLeftMotor.setVelocity(offset.acquisitionTime);
                frontRightMotor.setVelocity(offset.acquisitionTime);
                backLeftMotor.setVelocity(offset.acquisitionTime);
                backRightMotor.setVelocity(offset.acquisitionTime);
            }


        }

        if (!(frontLeftMotor.isBusy() || frontRightMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy())){
            this.complete = true;
        }
    }


}
