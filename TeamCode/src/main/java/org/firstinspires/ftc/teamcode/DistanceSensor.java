package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensor {

    public DistanceSensor() {

    }

    public double getDistance() {
        return Robot.robotMap.distanceSensor.getDistance(DistanceUnit.METER);
    }
}
