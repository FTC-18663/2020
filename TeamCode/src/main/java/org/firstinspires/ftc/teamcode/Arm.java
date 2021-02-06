package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class Arm {


    private PID ourPID;
    private ElapsedTime runtime = new ElapsedTime();

    public double calcPID = 0;
    public double armError = 0;

    public Arm() {
        ourPID = new PID(0.4, 0.000, 0.00);
        runtime.reset();
        Robot.robotMap.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Robot.robotMap.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    public void setTarget(boolean button, int encTarget, double power) {
        double PID = 0.00;
        double error = 0.00;

        if(button) {
            PID = ourPID.calculatePID(encTarget, Robot.robotMap.arm.getCurrentPosition(), runtime.milliseconds());
            error = ourPID.getLastError();
            if(encTarget < Robot.robotMap.arm.getCurrentPosition()) {
                Robot.robotMap.arm.setTargetPosition(encTarget);
                Robot.robotMap.arm.setPower(power);
            } else if(encTarget > Robot.robotMap.arm.getCurrentPosition()) {
                Robot.robotMap.arm.setTargetPosition(encTarget);
                Robot.robotMap.arm.setPower(power*0.15);
            }

        }

        setCalcPID(PID);
        setArmError(error);

//        Robot.robotMap.arm.setTargetPosition((int) PID);
//        Robot.robotMap.arm.setPower(0.25);




    }


    public void stop() {
        Robot.robotMap.arm.setPower(0.00);
    }
}