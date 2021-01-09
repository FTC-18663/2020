package org.firstinspires.ftc.teamcode;


public class Arm {


    public Arm() {

    }

    public void init() {

    }


    public void setArm(double rt, double lt, double Speed){
        if(rt>.03) {
            Robot.robotMap.arm.setPower(rt*Speed);
        }
        if(lt>.03) {
            Robot.robotMap.arm.setPower(-lt*Speed);
        }
        else{
            Robot.robotMap.arm.setPower(0.00);
        }

        // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    public void stop() {
        Robot.robotMap.arm.setPower(0.00);
    }
}