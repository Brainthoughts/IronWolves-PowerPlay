package org.ironwolves.ftc.navutils;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.PositionCalculator;

import java.lang.reflect.Array;
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

    public void run(){
        for (Instruction i : instructions) {
            if (!i.isComplete()){
                i.execute(posCalc);
                break;
            }
        }
    }



}
