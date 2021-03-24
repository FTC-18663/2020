package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.helpers.Constants;

public class Stage extends OpMode {

    private Servo stage0 = hardwareMap.get(Servo.class, Constants.Stage.STAGE0);
    private Servo stage1 = hardwareMap.get(Servo.class, Constants.Stage.STAGE1);

    public Stage() {

        stage0.setDirection(Servo.Direction.FORWARD);
        stage1.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }


    public void sweeper(boolean button,boolean button2) {
        if (button) {
            stage0.setPosition(0.25);
        }
        if (button2) {
            stage1.setPosition(0.25);
        }
    }

}