package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

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
            if (!i.complete){
                i.execute();
                break;
            }
        }
    }

    public static class Instruction {
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

        public Instruction(Code instructionCode, Object[] parameters) {
            this.instructionCode = instructionCode;
            this.parameters = parameters;
        }

        public void execute(){
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
            if (firstIteration){
                //calculate and set new position for wheel encoders
            }
        }


    }
    
}
