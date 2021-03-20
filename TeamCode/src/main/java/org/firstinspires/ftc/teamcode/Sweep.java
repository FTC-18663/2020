package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class Sweep {

    public Sweep() {
        Robot.robotMap.stage0.setDirection(Servo.Direction.FORWARD);
        Robot.robotMap.stage1.setDirection(Servo.Direction.FORWARD);
        ///Robot.robotMap.stage0.scaleRange(0,180);
    }

    public void sweepMove(double position) {

        if (position > -30) {
            Robot.robotMap.stage0.setPosition(0.5);
            Robot.robotMap.stage1.setPosition(0.5);
        } else {
            Robot.robotMap.stage0.setPosition(0.1);
            Robot.robotMap.stage1.setPosition(0.1);
        }
    }

    public void sweeper(boolean button) {
        if(button) {
            Robot.robotMap.stage0.setPosition(1.0);
            Robot.robotMap.stage1.setPosition(1.0);
        } else {
            Robot.robotMap.stage0.setPosition(0.0);
            Robot.robotMap.stage1.setPosition(0.0);
        }
    }
}
