package org.firstinspires.ftc.teamcode.subsystems;


import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotArmMode;

public class Shooter {


    public Shooter() {

    }

    public void setTarget(double trigger, double power) {
        if(trigger > 0.15) {
            Robot.robot.shooter0.setPower(power);
            Robot.robot.shooter1.setPower(power);
        } else {
            stop();
        }
    }

    public void stop() {
        Robot.robot.shooter0.setPower(0);
        Robot.robot.shooter1.setPower(0);
    }



}
