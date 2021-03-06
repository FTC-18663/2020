package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class Arm {


    private int launchPosition = -130;
    private int launchStopShort = -343;
    private int launchStopLong = -343;

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

        Robot.robotMap.arm0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Robot.robotMap.arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Robot.robotMap.arm0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.robotMap.arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Robot.robotMap.arm0.setDirection(DcMotorSimple.Direction.REVERSE);
        Robot.robotMap.arm1.setDirection(DcMotorSimple.Direction.REVERSE);

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


    public void setArm(boolean up, boolean down, boolean lb, boolean rb, double speed){

        double launchSpeed = speed * 1.4;
        double upSpeed = speed * 0.25;
        double downSpeed = speed * 0.25;


        if(up) {
            if(Robot.robotMap.arm0.getCurrentPosition() <= launchPosition) {
                stop();
            } else {

                //arm going up to launch position
                Robot.robotMap.arm0.setPower(-upSpeed);
                Robot.robotMap.arm1.setPower(upSpeed);
            }
        } else if(down) {
            Robot.robotMap.arm0.setPower(downSpeed);
            Robot.robotMap.arm1.setPower(-downSpeed);
        } else if (lb) {
            if(Robot.robotMap.arm0.getCurrentPosition() <= launchStopLong) {
                stop();
            } else {
                // set launch power
                Robot.robotMap.arm0.setPower(-launchSpeed);
                Robot.robotMap.arm1.setPower(launchSpeed);
            }
        } else if(rb) {
            if(Robot.robotMap.arm0.getCurrentPosition() <= launchStopShort) {
                stop();
            } else {
                // set launch power
                Robot.robotMap.arm0.setPower(-launchSpeed);
                Robot.robotMap.arm1.setPower(launchSpeed);
            }
        } else{
            if(Robot.robotMap.reset.isPressed()) {
                Robot.robotMap.arm0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Robot.robotMap.arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                Robot.robotMap.arm0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Robot.robotMap.arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            PID = ourPID.calculatePID(encTarget, Robot.robotMap.arm0.getCurrentPosition(), runtime.milliseconds());
            error = ourPID.getLastError();
            if(encTarget < Robot.robotMap.arm0.getCurrentPosition()) {
                Robot.robotMap.arm0.setTargetPosition(encTarget);
                Robot.robotMap.arm0.setPower(power);



            } else if(encTarget > Robot.robotMap.arm0.getCurrentPosition()) {
                Robot.robotMap.arm0.setTargetPosition(encTarget);
                Robot.robotMap.arm0.setPower(power*0.15);

                Robot.robotMap.arm1.setPower(Robot.robotMap.arm0.getPower());
            }

        }

        setCalcPID(PID);
        setArmError(error);

//        Robot.robotMap.arm.setTargetPosition((int) PID);
//        Robot.robotMap.arm.setPower(0.25);

    }

    public void resetOffset(boolean reset) {
        if(reset) {
            Robot.robotMap.arm0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Robot.robotMap.arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            Robot.robotMap.arm0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Robot.robotMap.arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }else {

        }
    }

    public void stop() {
        Robot.robotMap.arm0.setPower(0.00);
        Robot.robotMap.arm1.setPower(0.00);
    }
}