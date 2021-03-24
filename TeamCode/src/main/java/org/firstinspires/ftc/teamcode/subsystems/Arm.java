package org.firstinspires.ftc.teamcode.subsystems;


import android.text.method.Touch;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.helpers.Constants;
import org.firstinspires.ftc.teamcode.helpers.PID;

import java.nio.charset.CharsetDecoder;

public class Arm extends OpMode {

    private int launchPosition = -130;
    private int launchStopShort = -343;
    private int launchStopLong = -343;

    private DcMotor arm0 = hardwareMap.get(DcMotor.class, Constants.Arm.ARM0);
    private DcMotor arm1= hardwareMap.get(DcMotor.class,Constants.Arm.ARM1);
    private TouchSensor reset = hardwareMap.get(TouchSensor.class,Constants.Sensors.ARM_RESET);


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

        arm0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm0.setDirection(DcMotorSimple.Direction.REVERSE);
        arm1.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void init() {

        telemetry.addData("Position", arm0.getCurrentPosition());

    }

    @Override
    public void loop() {

        telemetry.addData("Position0", arm0.getCurrentPosition());
        telemetry.addData("Position1", arm1.getCurrentPosition());
        telemetry.addData("PID", getCalcPID());
        telemetry.addData("Error", getArmError());
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

        boolean beforeTarget = -arm1.getCurrentPosition() <= launchStopShort;


        if(up) {
            if (-arm1.getCurrentPosition() <= launchPosition) {
                stop();
            } else {

                //arm going up to launch position
                arm0.setPower(-upSpeed);
                arm1.setPower(upSpeed);
            }
        } else if(down) {
            arm0.setPower(downSpeed);
            arm1.setPower(-downSpeed);
        } else if (lb) {
            if (beforeTarget) {
                stop();
            } else {
                // set launch power
                arm0.setPower(-launchSpeed);
                arm1.setPower(launchSpeed);
            }
        } else if(rb) {
            if (beforeTarget) {
                stop();
            } else {
                // set launch power
                arm0.setPower(-launchSpeed);
                arm1.setPower(launchSpeed);
            }
        } else{
            if(reset.isPressed()) {
                arm0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                arm0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            PID = ourPID.calculatePID(encTarget, arm0.getCurrentPosition(), runtime.milliseconds());
            error = ourPID.getLastError();
            if(encTarget < arm0.getCurrentPosition()) {
                arm0.setTargetPosition(encTarget);
                arm0.setPower(power);



            } else if(encTarget > arm0.getCurrentPosition()) {
                arm0.setTargetPosition(encTarget);
                arm0.setPower(power*0.15);

                arm1.setPower(arm0.getPower());
            }

        }

        setCalcPID(PID);
        setArmError(error);

//        Robot.robotMap.arm.setTargetPosition((int) PID);
//        Robot.robotMap.arm.setPower(0.25);

    }

    public void resetOffset(boolean reset) {
        if(reset) {
            arm0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            arm0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }else {

        }
    }

    public void stop() {
        arm0.setPower(0.00);
        arm1.setPower(0.00);
    }
}