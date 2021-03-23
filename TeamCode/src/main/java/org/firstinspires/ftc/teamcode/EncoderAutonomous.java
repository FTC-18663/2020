package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Encoder Autonomous", group="Autonomous")
//@Disabled
public class EncoderAutonomous extends LinearOpMode
{
    DcMotor leftMotorF;
    DcMotor rightMotorF;
    DcMotor leftMotorR;
    DcMotor rightMotorR;

    @Override
    public void runOpMode() throws InterruptedException
    {
        leftMotorF = hardwareMap.dcMotor.get("left_drive0");
        rightMotorF = hardwareMap.dcMotor.get("right_drive1");
        leftMotorR = hardwareMap.dcMotor.get("left_drive2");
        rightMotorR = hardwareMap.dcMotor.get("right_drive3");



        // You will need to set this based on your robot's
        // gearing to get forward control input to result in
        // forward motion.
        leftMotorF.setDirection(DcMotor.Direction.REVERSE);
        leftMotorR.setDirection(DcMotor.Direction.REVERSE);

        // reset encoder counts kept by motors.
        leftMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set motors to run forward for 5000 encoder counts.
        leftMotorF.setTargetPosition(5000);
        rightMotorF.setTargetPosition(5000);
        leftMotorR.setTargetPosition(5000);
        rightMotorR.setTargetPosition(5000);

        // set motors to run to target encoder position and stop with brakes on.
        leftMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        // set both motors to 25% power. Movement will start. Sign of power is
        // ignored as sign of target encoder position controls direction when
        // running to position.

        leftMotorF.setPower(0.25);
        rightMotorF.setPower(0.25);
        leftMotorR.setPower(0.25);
        rightMotorR.setPower(0.25);

        // wait while opmode is active and left motor is busy running to position.

        while (opModeIsActive() && leftMotorF.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            telemetry.addData("encoder-fwd-left", leftMotorF.getCurrentPosition() + "  busy=" + leftMotorF.isBusy());
            telemetry.addData("encoder-fwd-right", rightMotorF.getCurrentPosition() + "  busy=" + rightMotorF.isBusy());
            telemetry.update();
            idle();
        }

        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.

        leftMotorF.setPower(0.0);
        rightMotorF.setPower(0.0);
        leftMotorR.setPower(0.0);
        rightMotorR.setPower(0.0);

        // wait 5 sec to you can observe the final encoder position.

        resetStartTime();

        while (opModeIsActive() && getRuntime() < 5)
        {
            telemetry.addData("encoder-fwd-left-end", leftMotorF.getCurrentPosition());
            telemetry.addData("encoder-fwd-right-end", rightMotorF.getCurrentPosition());
            telemetry.update();
            idle();
        }

        // From current position back up to starting point. In this example instead of
        // having the motor monitor the encoder we will monitor the encoder ourselves.

        leftMotorF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotorF.setTargetPosition(0);
        rightMotorF.setTargetPosition(0);
        leftMotorR.setTargetPosition(0);
        rightMotorR.setTargetPosition(0);

        // Power sign matters again as we are running without encoder.
        leftMotorF.setPower(-0.25);
        rightMotorF.setPower(-0.25);
        leftMotorR.setPower(-0.25);
        rightMotorR.setPower(-0.25);

        while (opModeIsActive() && leftMotorF.getCurrentPosition() > leftMotorF.getTargetPosition())
        {
            telemetry.addData("encoder-back-left", leftMotorF.getCurrentPosition());
            telemetry.addData("encoder-back-right", rightMotorF.getCurrentPosition());
            telemetry.update();
            idle();
        }

        // set motor power to zero to stop motors.

        leftMotorF.setPower(0.0);
        rightMotorF.setPower(0.0);
        leftMotorR.setPower(0.0);
        rightMotorR.setPower(0.0);

        resetStartTime();

        while (opModeIsActive() && getRuntime() < 5)
        {
            telemetry.addData("encoder-back-left-end", leftMotorF.getCurrentPosition());
            telemetry.addData("encoder-back-right-end", rightMotorF.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }
}