package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;


public class AutonomousNavigator {

    private PositionCalculator posCalc;
    private Position robotSize;
    GameField gameField;

    public AutonomousNavigator(PositionCalculator _posCalc, Position _robotSize, GameField _gameField) {
        this.posCalc = _posCalc;
        this.robotSize = _robotSize;
        this.gameField = _gameField;
    }

//    public calculateEncoderPosition(){
//
//    }
}
