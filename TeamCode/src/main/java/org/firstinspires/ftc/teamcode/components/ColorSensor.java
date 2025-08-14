package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.components.GlobalData.SampleColor;

public class ColorSensor {
    private static final float GAIN = 2f;

    private final NormalizedColorSensor colorSensor;
    private final DistanceSensor distanceSensor;
    private final double SAMPLE_DETECTION_RANGE = 7;
    private final double RED_TO_GREEN_YELLOW_RATIO = 1.5;

    private final double DATA_TRACKING_LOOPS_DELAY = 10;
    private double loop_tracker;

    private boolean cashedIsSampleDetected;
    private SampleColor cashedSampleColor;

    public ColorSensor(NormalizedColorSensor colorSensor) {
        this.colorSensor = colorSensor;
        this.distanceSensor = (DistanceSensor) colorSensor;
        this.colorSensor.setGain(GAIN);

        cashedSampleColor = SampleColor.UNKNOWN;
        cashedIsSampleDetected = false;

        loop_tracker = 0;
    }

    private double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }

    private float getRed() {
        return colorSensor.getNormalizedColors().red;
    }

    private float getGreen() {
        return colorSensor.getNormalizedColors().green;
    }

    private float getBlue() {
        return colorSensor.getNormalizedColors().blue;
    }


    private boolean determineIsSampleDetected() {
        return getDistance() <= SAMPLE_DETECTION_RANGE;
    }
    private SampleColor determineSampleColor() {
        if (getBlue() > getRed() && getBlue() > getGreen()) {
            return SampleColor.BLUE;
        }
        else if (getRed() / getGreen() > RED_TO_GREEN_YELLOW_RATIO) {
            return SampleColor.RED;
        }
        else {
            return SampleColor.YELLOW;
        }
    }
    public void operate() {
        if (loop_tracker >= DATA_TRACKING_LOOPS_DELAY) {
            determineSampleColor();
            determineIsSampleDetected();
            loop_tracker = 0;
        }
        loop_tracker++;
    }

    public boolean isSampleDetected(){
        return cashedIsSampleDetected;
    }

    public SampleColor getSampleColor() {
        return cashedSampleColor;
    }
}
