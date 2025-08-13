package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.GlobalData.SampleColor;

public class ColorSensor {
    private static final float GAIN = 2f;

    private final NormalizedColorSensor colorSensor;
    private final DistanceSensor distanceSensor;

    public ColorSensor(NormalizedColorSensor colorSensor) {
        this.colorSensor = colorSensor;
        this.distanceSensor = (DistanceSensor) colorSensor;
        this.colorSensor.setGain(GAIN);
    }

    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }

    public float getRed() {
        return colorSensor.getNormalizedColors().red;
    }

    public float getGreen() {
        return colorSensor.getNormalizedColors().green;
    }

    public float getBlue() {
        return colorSensor.getNormalizedColors().blue;
    }


    public boolean isSampleDetected() {
        return false;
    }
    public SampleColor getSampleColor() {
        return SampleColor.UNKNOWN;
    }
}
