package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.helpers.Constants;

public class Stage extends OpMode {

    private Servo stage0 = null;
    private Servo stage1 = null;

    public Stage(com.qualcomm.robotcore.hardware.HardwareMap hM) {

        stage0 = hM.get(Servo.class, Constants.Stage.STAGE0);
        stage1 = hM.get(Servo.class, Constants.Stage.STAGE1);


    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }


    public void stage(boolean button,boolean button2) {
        stage0.setDirection(Servo.Direction.FORWARD);
        stage1.setDirection(Servo.Direction.FORWARD);

        if (button) {
            stage0.setPosition(0.25);
        }
        if (button2) {
            stage1.setPosition(0.25);
        }
    }

}