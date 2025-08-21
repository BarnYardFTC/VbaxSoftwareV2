package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.GlobalData.SampleColor;

public class ColorSensorEx {

    /*───────────────────────────────*
     *        CONFIGURATION          *
     *───────────────────────────────*/
    private static final float GAIN = 2.0f;
    private static final double RED_TO_GREEN_MIN_YELLOW_RATIO = 1.5;
    private static final double SAMPLE_DETECTED_MAX_DISTANCE_CM = 7.0;
    private static final int LOOP_DELAY = 10;  // Number of loops between sensor updates

    /*───────────────────────────────*
     *      HARDWARE REFERENCES      *
     *───────────────────────────────*/
    private final NormalizedColorSensor colorSensor;
    private final DistanceSensor distanceSensor;

    /*───────────────────────────────*
     *      CACHED SENSOR DATA       *
     *───────────────────────────────*/
    private int loopCounter = 0;
    private SampleColor cachedSampleColor;
    private boolean cachedSampleDetected;

    /*───────────────────────────────*
     *          CONSTRUCTOR          *
     *───────────────────────────────*/
    public ColorSensorEx(NormalizedColorSensor colorSensor) {
        this.colorSensor = colorSensor;
        this.distanceSensor = (DistanceSensor) colorSensor;

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        this.colorSensor.setGain(GAIN);

        cachedSampleColor = SampleColor.UNKNOWN;
        cachedSampleDetected = false;
    }

    /*───────────────────────────────*
     *        MAIN OPERATE LOOP      *
     *───────────────────────────────*/
    /**
     * Call this method once per main robot loop to update cached sensor data periodically.
     */
    public void operate() {
        loopCounter++;
        if (loopCounter >= LOOP_DELAY) {
            loopCounter = 0;
            updateSensorData();
        }
    }

    /*───────────────────────────────*
     *      SENSOR DATA UPDATES     *
     *───────────────────────────────*/
    private void updateSensorData() {
        cachedSampleDetected = getDistanceCm() < SAMPLE_DETECTED_MAX_DISTANCE_CM;

        if (!cachedSampleDetected) {
            cachedSampleColor = SampleColor.UNKNOWN;
            return;
        }

        cachedSampleColor = determineSampleColor();
    }

    private SampleColor determineSampleColor() {
        double red   = colorSensor.getNormalizedColors().red;
        double green = colorSensor.getNormalizedColors().green;
        double blue  = colorSensor.getNormalizedColors().blue;

        if (blue > green && blue > red) {
            return SampleColor.BLUE;
        }
        if (red / green > RED_TO_GREEN_MIN_YELLOW_RATIO) {
            return SampleColor.RED;
        }
        return SampleColor.YELLOW;
    }

    /*───────────────────────────────*
     *      PUBLIC SENSOR ACCESS     *
     *───────────────────────────────*/
    public double getDistanceCm() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }

    /**
     * Returns whether a sample is currently detected (cached, updated periodically).
     */
    public boolean isSampleDetected() {
        return cachedSampleDetected;
    }

    /**
     * Returns the current sample color (cached, updated periodically).
     */
    public SampleColor getSampleColor() {
        return cachedSampleColor;
    }
}
