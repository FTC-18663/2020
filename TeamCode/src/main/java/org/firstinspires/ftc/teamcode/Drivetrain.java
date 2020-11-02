package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class Drivetrain {
//    private DcMotor leftDriveF = hardwareMap.get(DcMotor.class, "left_drive0");
//    private DcMotor rightDriveF = hardwareMap.get(DcMotor.class, "right_drive1");
//    private DcMotor leftDriveR = hardwareMap.get(DcMotor.class, "left_drive2");
//    private DcMotor rightDriveR = hardwareMap.get(DcMotor.class, "right_drive3");

    public Drivetrain() {

    }

    public void init() {
//        leftDriveF = hardwareMap.dcMotor.get("left_drive0");
//        rightDriveF = hardwareMap.dcMotor.get("right_drive1");
//        leftDriveR = hardwareMap.dcMotor.get("left_drive2");
//        rightDriveR = hardwareMap.dcMotor.get("right_drive3");
//
//        leftDriveF.setDirection(DcMotor.Direction.FORWARD);
//        rightDriveF.setDirection(DcMotor.Direction.REVERSE);
//        leftDriveR.setDirection(DcMotor.Direction.FORWARD);
//        rightDriveR.setDirection(DcMotor.Direction.REVERSE);
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
