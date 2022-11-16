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

    private DcMotorEx frontLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx backRightMotor;

    private final double ppr = ((1+(46.0/11)) * 28);
    private final double wheelRadius = 0.048; //in meters
    private final double gearRatio = 10d/14d;

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
