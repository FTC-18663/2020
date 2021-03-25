/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.text.method.Touch;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.internal.network.ControlHubDeviceNameManager;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareMap
{
    /* Public OpMode members. */
    public DcMotor leftDriveF = null;//  = opMode.hardwareMap.get(DcMotor.class, "left_drive0");
    public DcMotor rightDriveF = null;// = opMode.hardwareMap.get(DcMotor.class, "right_drive1");
    public DcMotor leftDriveR = null;// = opMode.hardwareMap.get(DcMotor.class, "left_drive2");
    public DcMotor rightDriveR = null;// = opMode.hardwareMap.get(DcMotor.class, "right_drive2");
    public DcMotor arm0 = null;
    public DcMotor arm1 = null;
    public TouchSensor reset = null;
    public DistanceSensor distanceSensor = null;
    public DcMotor shooter0 = null;
    public DcMotor shooter1 = null;
    public DcMotor intake = null;
    public Servo stage1 = null;
    public Servo stage2 = null;
    //public DcMotor wrist = null;
    //public DcMotor carousel = null;

    /* local OpMode members. */
    com.qualcomm.robotcore.hardware.HardwareMap hwMap = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareMap(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(com.qualcomm.robotcore.hardware.HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDriveF  = hwMap.get(DcMotor.class, "left_drive0");
        rightDriveF = hwMap.get(DcMotor.class, "right_drive1");
        leftDriveR = hwMap.get(DcMotor.class, "left_drive2");
        rightDriveR = hwMap.get(DcMotor.class, "right_drive3");
        arm0 = hwMap.get(DcMotor.class, "arm0");
        arm1 = hwMap.get(DcMotor.class,"arm1");
        reset = hwMap.get(TouchSensor.class,"reset");
        distanceSensor = hwMap.get(DistanceSensor.class, "sense");

        shooter0 = hwMap.get(DcMotor.class, "shooter0");
        shooter1 = hwMap.get(DcMotor.class, "shooter1");
        intake = hwMap.get(DcMotor.class, "intake");

        stage1 = hwMap.get(Servo.class, "stage1");
        stage2 = hwMap.get(Servo.class, "stage2");

        //wrist  = hwMap.get(DcMotor.class, "wrist");


        //leftArm    = hwMap.get(DcMotor.class, "left_arm");
        //motorFrontLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        //motorFrontRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftDriveF.setPower(0.00);
        rightDriveF.setPower(0.00);
        leftDriveR.setPower(0.00);
        rightDriveR.setPower(0.00);
        arm0.setPower(0.0);
        arm1.setPower(0.0);
        //wrist.setPower(0.0);

        //leftArm.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
//        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
//        leftClaw  = hwMap.get(Servo.class, "left_hand");
//        rightClaw = hwMap.get(Servo.class, "right_hand");
//        leftClaw.setPosition(MID_SERVO);
//        rightClaw.setPosition(MID_SERVO)
    }
}