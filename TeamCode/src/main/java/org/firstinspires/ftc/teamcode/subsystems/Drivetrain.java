package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot;

public class Drivetrain {

    private static final double COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    private static final double DRIVE_SPEED             = 0.6;
    private static final double TURN_SPEED              = 0.5;

    public Drivetrain() {

        Robot.robot.leftDriveF.setDirection(DcMotorSimple.Direction.FORWARD);
        Robot.robot.rightDriveF.setDirection(DcMotorSimple.Direction.FORWARD);
        Robot.robot.leftDriveR.setDirection(DcMotorSimple.Direction.REVERSE);
        Robot.robot.rightDriveR.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void init() {

    }


    public void setDrive(double fwd, double rot, double maxSpeed){
        double leftPower = (fwd - rot) * maxSpeed;
        double rightPower = (fwd + rot) * maxSpeed;

        Robot.robot.leftDriveF.setPower(leftPower);
        Robot.robot.rightDriveF.setPower(rightPower);
        Robot.robot.leftDriveR.setPower(leftPower);
        Robot.robot.rightDriveR.setPower(rightPower);

       // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    public void setEncoderMode() {
        Robot.robot.leftDriveF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.robot.rightDriveF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.robot.leftDriveR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.robot.rightDriveR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Robot.robot.leftDriveF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.robot.rightDriveF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.robot.leftDriveR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.robot.rightDriveR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active

            // Determine new target position, and pass to motor controller
            newLeftTarget = Robot.robot.leftDriveF.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = Robot.robot.rightDriveF.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            Robot.robot.leftDriveF.setTargetPosition(newLeftTarget);
            Robot.robot.rightDriveF.setTargetPosition(newRightTarget);
            Robot.robot.leftDriveR.setTargetPosition(newLeftTarget);
            Robot.robot.rightDriveR.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            Robot.robot.leftDriveF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Robot.robot.rightDriveF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            Robot.runtime.reset();
            Robot.robot.leftDriveF.setPower(Math.abs(speed));
            Robot.robot.rightDriveF.setPower(Math.abs(speed));
            Robot.robot.leftDriveR.setPower(Math.abs(speed));
            Robot.robot.rightDriveR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((Robot.runtime.seconds() < timeoutS) && (Robot.robot.leftDriveF.isBusy() && Robot.robot.rightDriveF.isBusy())) {

                // Display it for the driver.
//                Robot.telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
//                Robot.telemetry.addData("Path2",  "Running at %7d :%7d", Robot.robot.leftDriveF.getCurrentPosition(), Robot.robot.rightDriveF.getCurrentPosition());
//                telemetry.update();
            }

            // Stop all motion;
            stop();

            // Turn off RUN_TO_POSITION
            Robot.robot.leftDriveF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Robot.robot.rightDriveF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Robot.robot.leftDriveR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Robot.robot.rightDriveR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move

    }



    public void stop() {
        Robot.robot.leftDriveF.setPower(0.00);
        Robot.robot.rightDriveF.setPower(0.00);
        Robot.robot.leftDriveR.setPower(0.00);
        Robot.robot.rightDriveR.setPower(0.00);
    }
}
