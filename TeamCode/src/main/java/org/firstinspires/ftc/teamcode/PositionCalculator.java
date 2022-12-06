package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class PositionCalculator {

    public enum MovementMode {
        Vertical,
        Horizontal,
        Rotational
    }

    private MovementMode currentMode = MovementMode.Vertical;

    public Position position = new Position(DistanceUnit.METER,0,0,0, 1);

    private final DcMotorEx frontLeftMotor;
    private final DcMotorEx frontRightMotor;
    private final DcMotorEx backLeftMotor;
    private final DcMotorEx backRightMotor;

    private final double ppr = Config.Hardware.Motor.driveMotorPPR;
    private final double wheelRadius = Config.Hardware.Wheel.wheelRadius; //in meters
    private final double gearRatio = Config.Hardware.Wheel.gearRatio;

    private int lastPos;


    public PositionCalculator(DcMotorEx _frontLeftMotor, DcMotorEx _frontRightMotor, DcMotorEx _backLeftMotor, DcMotorEx _backRightMotor){
        frontLeftMotor = _frontLeftMotor;
        frontRightMotor = _frontRightMotor;
        backLeftMotor = _backLeftMotor;
        backRightMotor = _backRightMotor;
    }

    public Position getPosition(){
        updatePosition();
        return position;
    }

    public void setMode(MovementMode mode){
        updatePosition();
        currentMode = mode;
    }

    private void updatePosition(){
        switch (currentMode){
            case Vertical:
                position.y += motorRotationToDistance(frontLeftMotor.getCurrentPosition()-lastPos);
                break;
            case Horizontal:
                position.x += motorRotationToDistance(frontLeftMotor.getCurrentPosition()-lastPos);
                break;
        }
        lastPos = frontLeftMotor.getCurrentPosition();
    }

    public double motorRotationToDistance(int deltaEncoder){
        return deltaEncoder*gearRatio/ppr*2*Math.PI*wheelRadius;
    }

    public int distanceToMotorRotation(double distance){
        return (int) ((distance*ppr)/(gearRatio*2*Math.PI*wheelRadius));
    }
}
