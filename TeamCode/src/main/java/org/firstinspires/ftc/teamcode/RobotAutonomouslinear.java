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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

@Autonomous(name="RobotAutoL", group="Iterative Opmode")
//@Disabled
public class RobotAutonomouslinear extends LinearOpMode
{

    //HardwareMap robot = new HardwareMap();



    // Declare OpMode members.
    public static ElapsedTime runtime = new ElapsedTime();

    public double time = 0.00;

    //public static HardwareMap robotMap = new HardwareMap();
    public static Drivetrain m_drive;
    public static Arm m_arm;
    public static Distance m_distanceSensor;
    public static Stage m_stage;
    public RobotIO m_io;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");

        m_drive = new Drivetrain(hardwareMap);
        m_arm = new Arm(hardwareMap);
        m_distanceSensor = new Distance(hardwareMap);
        m_stage = new Stage(hardwareMap);

        m_io = new RobotIO();
        m_io.init();

        telemetry.addData("Status", "Initialized");

        runtime.reset();
        time = runtime.seconds() + 30;
        waitForStart();


            m_drive.encoderDrive(1.0,15, 15, 15);

            telemetry.addData("Position0", m_arm.getarm0Position());
            telemetry.addData("Position1", m_arm.getarm1Position());


    }

    /*
     * Code to run ONCE when the driver hits INIT
     */






        //drive.init();





        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery


        // Tell the driver that initialization is complete.



    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */

    /*
     * Code to run ONCE when the driver hits PLAY
     */


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */











//        telemetry.addData("Position0", Robot.robot.arm0.getCurrentPosition());
//        telemetry.addData("Position1", Robot.robot.arm1.getCurrentPosition());
//        telemetry.addData("PID", m_arm.getCalcPID());
//        telemetry.addData("Error", m_arm.getArmError());
//        telemetry.addData("Distance", m_distanceSensor.getDistance() + "M");
//        telemetry.addData("Avg", m_arm.getAvg());
        // Show the elapsed game time and wheel power.
        //telemetry.addData("Status", "Run Time: " + runtime.toString());
//

}
