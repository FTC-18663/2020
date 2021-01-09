package org.firstinspires.ftc.teamcode;


public class Wrist {


    public Wrist() {

    }

    public void init() {

    }


    public void setWrist(boolean up, boolean down, double Speed){
        if(up) {
            Robot.robotMap.wrist.setPower(Speed);
        }
        if(down) {
            Robot.robotMap.wrist.setPower(Speed);
        }
        else{
            Robot.robotMap.wrist.setPower(0.00);
        }

        // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    public void stop() {
        Robot.robotMap.wrist.setPower(0.00);
    }
}