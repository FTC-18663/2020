package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class Sweep {

    public Sweep() {
        Robot.robotMap.stage0.setDirection(Servo.Direction.FORWARD);
        Robot.robotMap.stage1.setDirection(Servo.Direction.REVERSE);
    }


    public void sweeper(boolean button,boolean button2) {
        if (button) {
            Robot.robotMap.stage0.setPosition(0.25);
        }
        if (button2) {
            Robot.robotMap.stage1.setPosition(0.25);
        }
    }

}
