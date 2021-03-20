package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
public class Robot extends OpMode {
    public static HardwareMap robot = new HardwareMap();
    public static ElapsedTime runtime = new ElapsedTime();

    public Robot(com.qualcomm.robotcore.hardware.HardwareMap hardwareMap) {
        robot.init(hardwareMap);
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }

}