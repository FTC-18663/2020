package org.firstinspires.ftc.teamcode;


public class Drivetrain {


    public Drivetrain() {

    }

    public void init() {

    }


    public void setDrive(double fwd, double rot, double maxSpeed){
        double leftPower = (fwd + rot) * maxSpeed;
        double rightPower = (fwd - rot) * maxSpeed;

        Robot.robotMap.leftDriveF.setPower(leftPower);
        Robot.robotMap.rightDriveF.setPower(-rightPower);
        Robot.robotMap.leftDriveR.setPower(leftPower);
        Robot.robotMap.rightDriveR.setPower(-rightPower);

       // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    public void stop() {
        Robot.robotMap.leftDriveF.setPower(0.00);
        Robot.robotMap.rightDriveF.setPower(0.00);
        Robot.robotMap.leftDriveR.setPower(0.00);
        Robot.robotMap.rightDriveR.setPower(0.00);
    }
}
