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

import com.qualcomm.hardware.rev.RevSPARKMini;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Distance;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Stage;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Robot", group="Iterative Opmode")
//@Disabled
public class Robot extends OpMode
{

    //HardwareMap robot = new HardwareMap();



    // Declare OpMode members.
    public static ElapsedTime runtime = new ElapsedTime();

    //public static HardwareMap robotMap = new HardwareMap();
    public static Drivetrain m_drive;
    public static Arm m_arm;
    public static Distance m_distanceSensor;
    public static Stage m_stage;
    public static VoltageSensor m_voltage;

    public RobotIO m_io;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {



        telemetry.addData("Status", "Initialized");


        m_drive = new Drivetrain(hardwareMap);
        m_arm = new Arm(hardwareMap);
        m_distanceSensor = new Distance(hardwareMap);
        m_stage = new Stage(hardwareMap);
        //m_voltage = hardwareMap.voltageSensor.get("armspark");



        m_io = new RobotIO();
        m_io.init();

        //drive.init();
        m_drive.stop();




        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        //telemetry.addData("Position", Robot.robot.arm0.getCurrentPosition());
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        //m_drive.setEncoderMode();
        if(gamepad2.right_bumper) {
            m_drive.setDrive(gamepad2.left_stick_y, gamepad2.right_stick_x, 1.00);
        } else {
            m_drive.setDrive(gamepad2.left_stick_y, gamepad2.right_stick_x, 0.40);
        }

        m_arm.setArm(gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.left_bumper, gamepad1.right_bumper,0.5);

        m_arm.resetOffset(gamepad1.dpad_left);
        m_stage.stage(gamepad1.a, gamepad1.b);


        telemetry.addData("Position0", m_arm.getarm0Position());
        telemetry.addData("Position1", m_arm.getarm1Position());
        telemetry.addData("Data0", m_arm.getarm0Connection());
        telemetry.addData("Power0", m_arm.getarm0Power());





//        telemetry.addData("Position0", Robot.robot.arm0.getCurrentPosition());
//        telemetry.addData("Position1", Robot.robot.arm1.getCurrentPosition());
//        telemetry.addData("PID", m_arm.getCalcPID());
//        telemetry.addData("Error", m_arm.getArmError());
//        telemetry.addData("Distance", m_distanceSensor.getDistance() + "M");
//        telemetry.addData("Avg", m_arm.getAvg());
        // Show the elapsed game time and wheel power.
        //telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        //m_drive.stop();
    }

}
