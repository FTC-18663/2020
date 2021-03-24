package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.helpers.Constants;

public class Distance extends OpMode {

    private DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, Constants.Sensors.DISTANCE_SENSOR);

    public Distance() {

    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }

    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.METER);
    }
}
