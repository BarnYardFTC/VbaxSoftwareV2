package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Otos {
    private SparkFunOTOS otos;
    private double headingOffset;

    public Otos(SparkFunOTOS otos, double headingOffset) {
       this.otos = otos;
       initOtos();
       this.headingOffset = Math.toRadians(headingOffset);
    }
    private void initOtos() {
        otos.setLinearUnit(DistanceUnit.METER);
        otos.setAngularUnit(AngleUnit.RADIANS);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setOffset(offset);
        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);
        otos.calibrateImu();
        otos.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setPosition(currentPosition);
    }
    public void resetHeading() {
        otos.resetTracking();
    }
    public double getHeading() {
        return otos.getPosition().h + headingOffset;
    }
}