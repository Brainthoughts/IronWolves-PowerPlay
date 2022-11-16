package org.ironwolves.ftc.navutils;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.PositionCalculator;

public class Instruction {
    public enum Code {
        Move,
        Rotate,
        Sense,
        Claw,
        Lift
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
            case Sense:
                sense();
                break;
        }
        firstIteration = false;
    }

    private void sense() {

    }

    private void claw() {

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

        if (firstIteration){
            //calculate and set new position for wheel encoders

            int frontLeftEncoder = frontLeftMotor.getCurrentPosition();
            int frontRightEncoder = frontRightMotor.getCurrentPosition();
            int backLeftEncoder = backLeftMotor.getCurrentPosition();
            int backRightEncoder = backRightMotor.getCurrentPosition();

            frontLeftEncoder += posCalc.distanceToMotorRotation(offset.y);
            frontRightEncoder += posCalc.distanceToMotorRotation(offset.y);
            backLeftEncoder += posCalc.distanceToMotorRotation(offset.y);
            backRightEncoder += posCalc.distanceToMotorRotation(offset.y);

            frontLeftMotor.setTargetPosition(frontLeftEncoder);
            frontRightMotor.setTargetPosition(frontRightEncoder);
            backLeftMotor.setTargetPosition(backLeftEncoder);
            backRightMotor.setTargetPosition(backRightEncoder);

            frontLeftMotor.setVelocity(offset.acquisitionTime);
            frontRightMotor.setVelocity(offset.acquisitionTime);
            backLeftMotor.setVelocity(offset.acquisitionTime);
            backRightMotor.setVelocity(offset.acquisitionTime);

        }

        if (!(frontLeftMotor.isBusy() || frontRightMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy())){
            this.complete = true;
        }
    }


}
