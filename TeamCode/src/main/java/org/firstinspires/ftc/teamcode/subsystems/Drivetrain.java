package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.helpers.Constants;


public class Drivetrain extends OpMode {

    private static final double COUNTS_PER_MOTOR_REV    = 6000 ;    // eg: TETRIX Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION    = 40.0 ;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    private static final double DRIVE_SPEED             = 0.6;
    private static final double TURN_SPEED              = 0.5;

    private DcMotor leftDriveF  = hardwareMap.get(DcMotor.class, Constants.Drivetrain.LEFT_DRIVE_FRONT);
    private DcMotor rightDriveF = hardwareMap.get(DcMotor.class, Constants.Drivetrain.RIGHT_DRIVE_FRONT);
    private DcMotor leftDriveR = hardwareMap.get(DcMotor.class, Constants.Drivetrain.LEFT_DRIVE_REAR);
    private DcMotor rightDriveR = hardwareMap.get(DcMotor.class, Constants.Drivetrain.RIGHT_DRIVE_REAR);

    public Drivetrain() {

        leftDriveF.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDriveF.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDriveR.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDriveR.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }

    public void setDrive(double fwd, double rot, double maxSpeed){
        double leftPower = (fwd - rot) * maxSpeed;
        double rightPower = (fwd + rot) * maxSpeed;

        leftDriveF.setPower(leftPower);
        rightDriveF.setPower(rightPower);
        leftDriveR.setPower(leftPower);
        rightDriveR.setPower(rightPower);

       // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    public void setEncoderMode() {
        leftDriveF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDriveF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDriveF.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightDriveF.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            leftDriveF.setTargetPosition(newLeftTarget);
            rightDriveF.setTargetPosition(newRightTarget);
            leftDriveR.setTargetPosition(newLeftTarget);
            rightDriveR.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDriveF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            Robot.runtime.reset();
            leftDriveF.setPower(Math.abs(speed));
            rightDriveF.setPower(Math.abs(speed));
            leftDriveR.setPower(Math.abs(speed));
            rightDriveR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((Robot.runtime.seconds() < timeoutS) && (leftDriveF.isBusy() && rightDriveF.isBusy())) {

                // Display it for the driver.
//                Robot.telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
//                Robot.telemetry.addData("Path2",  "Running at %7d :%7d", Robot.robot.leftDriveF.getCurrentPosition(), Robot.robot.rightDriveF.getCurrentPosition());
//                telemetry.update();
            }

            // Stop all motion;
            stop();

            // Turn off RUN_TO_POSITION
            leftDriveF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDriveF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDriveR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDriveR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move

    }

    public void stop() {
        leftDriveF.setPower(0.00);
        rightDriveF.setPower(0.00);
        leftDriveR.setPower(0.00);
        rightDriveR.setPower(0.00);
    }
}