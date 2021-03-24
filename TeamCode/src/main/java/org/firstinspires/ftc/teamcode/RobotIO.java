package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static org.firstinspires.ftc.teamcode.Robot.m_arm;
import static org.firstinspires.ftc.teamcode.Robot.m_drive;

public class RobotIO extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        if(gamepad2.right_bumper) {
            m_drive.setDrive(gamepad2.left_stick_y, gamepad2.right_stick_x, 0.40);
        } else {
            m_drive.setDrive(gamepad2.left_stick_y, gamepad2.right_stick_x, 1.00);
        }



        m_arm.setArm(gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.left_bumper, gamepad1.right_bumper,0.5);

        m_arm.resetOffset(gamepad1.dpad_left);


    }
}
