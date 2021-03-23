package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {


    public Shooter() {

    }

    public void setTarget(double trigger, double power) {
        if(trigger > 0.15) {
            RobotTesting.shooter0.setPower(power);
            RobotTesting.shooter1.setPower(power);
        } else {
            stop();
        }

    }

    public void stop() {
        RobotTesting.shooter0.setPower(0);
        RobotTesting.shooter1.setPower(0);
    }



}