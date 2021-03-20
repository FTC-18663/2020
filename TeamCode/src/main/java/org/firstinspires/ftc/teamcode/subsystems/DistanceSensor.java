package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotArmMode;

public class DistanceSensor {

    public DistanceSensor() {

    }

    public double getDistance() {
        return Robot.robot.distanceSensor.getDistance(DistanceUnit.METER);
    }
}
