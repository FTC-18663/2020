package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.RobotArmMode;

public class Arm {

    private int launchPosition = -130;
    private int launchStopShort = -343;
    private int launchStopLong = -343;

    private int beforeTargetValue;

    private PID ourPID;
    private ElapsedTime runtime = new ElapsedTime();

    public double calcPID = 0;
    public double armError = 0;

    public Arm() {
        ourPID = new PID(0.4, 0.000, 0.00);
        runtime.reset();
        //Robot.robotMap.arm0.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        Robot.robotMap.arm0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Robot.robotMap.arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Robot.robot.arm0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Robot.robot.arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Robot.robot.arm0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.robot.arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Robot.robot.arm0.setDirection(DcMotorSimple.Direction.REVERSE);
        Robot.robot.arm1.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public double getCalcPID() {
        return this.calcPID;
    }

    public void setCalcPID(double pid) {
        this.calcPID = pid;
    }

    public void setArmError(double error) {
        this.armError = error;
    }

    public double getArmError() {
        return this.armError;
    }

    public double getAvg() {
        return beforeTargetValue;
    }


    public void setArm(boolean up, boolean down, boolean lb, boolean rb, double speed){

        double launchSpeed = speed * 1.6;
        double upSpeed = speed * 0.25;
        double downSpeed = speed * 0.25;

        //beforeTargetValue = -(Math.abs(Robot.r`.arm0.getCurrentPosition()) + Math.abs(Robot.robotMap.arm1.getCurrentPosition())) / 2;

        //boolean beforeTarget = beforeTargetValue <= launchStopShort;

        boolean beforeTarget = -Robot.robot.arm1.getCurrentPosition() <= launchStopShort;


        if(up) {
            if (-Robot.robot.arm1.getCurrentPosition() <= launchPosition) {
                stop();
            } else {

                //arm going up to launch position
                Robot.robot.arm0.setPower(-upSpeed);
                Robot.robot.arm1.setPower(upSpeed);
            }
        } else if(down) {
            Robot.robot.arm0.setPower(downSpeed);
            Robot.robot.arm1.setPower(-downSpeed);
        } else if (lb) {
            if (beforeTarget) {
                stop();
            } else {
                // set launch power
                Robot.robot.arm0.setPower(-launchSpeed);
                Robot.robot.arm1.setPower(launchSpeed);
            }
        } else if(rb) {
            if (beforeTarget) {
                stop();
            } else {
                // set launch power
                Robot.robot.arm0.setPower(-launchSpeed);
                Robot.robot.arm1.setPower(launchSpeed);
            }
        } else{
            if(Robot.robot.reset.isPressed()) {
                Robot.robot.arm0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Robot.robot.arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                Robot.robot.arm0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Robot.robot.arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                stop();
            }

        }


        // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    public void setTarget(boolean button, int encTarget, double power) {
        double PID = 0.00;
        double error = 0.00;

        if(button) {
            PID = ourPID.calculatePID(encTarget, Robot.robot.arm0.getCurrentPosition(), runtime.milliseconds());
            error = ourPID.getLastError();
            if(encTarget < Robot.robot.arm0.getCurrentPosition()) {
                Robot.robot.arm0.setTargetPosition(encTarget);
                Robot.robot.arm0.setPower(power);



            } else if(encTarget > Robot.robot.arm0.getCurrentPosition()) {
                Robot.robot.arm0.setTargetPosition(encTarget);
                Robot.robot.arm0.setPower(power*0.15);

                Robot.robot.arm1.setPower(Robot.robot.arm0.getPower());
            }

        }

        setCalcPID(PID);
        setArmError(error);

//        Robot.robotMap.arm.setTargetPosition((int) PID);
//        Robot.robotMap.arm.setPower(0.25);

    }

    public void resetOffset(boolean reset) {
        if(reset) {
            Robot.robot.arm0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Robot.robot.arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            Robot.robot.arm0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Robot.robot.arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }else {

        }
    }

    public void stop() {
        Robot.robot.arm0.setPower(0.00);
        Robot.robot.arm1.setPower(0.00);
    }
}