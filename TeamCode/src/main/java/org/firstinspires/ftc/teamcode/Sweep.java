package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class Sweep {

    public Sweep() {
        Robot.robotMap.sweep0.setDirection(Servo.Direction.FORWARD);
        Robot.robotMap.sweep1.setDirection(Servo.Direction.FORWARD);
    }

    public void sweepMove(double position) {

        if (position > -30) {
            Robot.robotMap.sweep0.setPosition(0.5);
            Robot.robotMap.sweep1.setPosition(0.5);
        } else {
            Robot.robotMap.sweep0.setPosition(0.1);
            Robot.robotMap.sweep1.setPosition(0.1);
        }
    }

    public void sweeper(boolean button) {
        if(button) {
            Robot.robotMap.sweep0.setPosition(1.0);
            Robot.robotMap.sweep1.setPosition(1.0);
        } else {
            Robot.robotMap.sweep0.setPosition(0.0);
            Robot.robotMap.sweep1.setPosition(0.0);
        }
    }
}
