package org.ironwolves.ftc.navutils;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

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
        addInstruction(new Instruction(Instruction.Code.Move, new Object[]{distance, motor1, motor2, motor3, motor4}));

    }

    public void claw(double position, Servo clawServo){
        addInstruction(new Instruction(Instruction.Code.Claw, new Object[]{position, clawServo}));
    }

    public void sleep(int sleepTimeMilliseconds){
        addInstruction(new Instruction(Instruction.Code.Sleep, new Object[]{System.currentTimeMillis() + sleepTimeMilliseconds}));
    }

    public void rotate(){
        throw new RuntimeException("Rotation Not yet implemented");
    }

    public void custom(Runnable code){
        Instruction i = new Instruction(Instruction.Code.Custom, new Object[]{code});
        addInstruction(i);
//        return i;
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
