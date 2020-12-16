package org.firstinspires.ftc.teamcode;


public class Intake {

    public Intake() {

    }

    public void init() {

    }


    public void setIntake(boolean x, double Speed){
        if(x) {
            Robot.robotMap.intake.setPower(Speed);
        }
        else{
            Robot.robotMap.intake.setPower(0.00);
        }

        // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    public void stop() {
        Robot.robotMap.intake.setPower(0.00);
    }
}