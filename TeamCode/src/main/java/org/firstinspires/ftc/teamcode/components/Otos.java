package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Otos {

    /* =========================
       HARDWARE REFERENCES
       ========================= */
    private final SparkFunOTOS otos;
    private final double headingOffset;

    /* =========================
       CONSTRUCTOR
       ========================= */
    public Otos(SparkFunOTOS otos, double headingOffset) {
        this.otos = otos;
        initOtos();
        this.headingOffset = headingOffset;
    }

    /* =========================
       PUBLIC METHODS
       ========================= */
    public void resetHeading() {
        otos.resetTracking();
    }

    public double getHeading() {
        return otos.getPosition().h + headingOffset;
    }

    /* =========================
       PRIVATE HELPERS
       ========================= */
    private void initOtos() {
        otos.setLinearUnit(DistanceUnit.METER);
        otos.setAngularUnit(AngleUnit.RADIANS);

        otos.setOffset(new SparkFunOTOS.Pose2D(0, 0, 0));
        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);

        otos.calibrateImu();
        otos.resetTracking();
        otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
    }
}
