package org.ironwolves.ftc.navutils;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.PositionCalculator;

import java.util.ArrayList;


public class AutonomousNavigator {

    private final PositionCalculator posCalc;
    private final ArrayList<Instruction> instructions;

    public AutonomousNavigator(PositionCalculator _posCalc) {
        this.posCalc = _posCalc;
        this.instructions = new ArrayList<>();
    }

    public void addInstruction(Instruction i){
        instructions.add(i);
    }

    public void move(Position distance, DcMotorEx motor1, DcMotorEx motor2, DcMotorEx motor3, DcMotorEx motor4){
        addInstruction(new Instruction(Instruction.Code.Move, new Object[]{new Position(DistanceUnit.METER, 0, 1, 0, 500), motor1, motor2, motor3, motor4}));

    }

    public void claw(double position, Servo clawServo){
        addInstruction(new Instruction(Instruction.Code.Claw, new Object[]{position, clawServo}));
    }

    public void run(){
        for (Instruction i : instructions) {
            if (!i.isComplete()){
                i.execute(posCalc);
                break;
            }
        }
    }



}
