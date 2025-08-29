package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.GlobalData.SampleColor;

public class ColorSensor {
    private static final float GAIN = 2f;

    private final NormalizedColorSensor colorSensor;
    private final DistanceSensor distanceSensor;
    private final double SAMPLE_DETECTION_RANGE = 7;
    private final double RED_TO_GREEN_YELLOW_RATIO = 1.5;



    public ColorSensor(NormalizedColorSensor colorSensor) {
        this.colorSensor = colorSensor;
        this.distanceSensor = (DistanceSensor) colorSensor;
        this.colorSensor.setGain(GAIN);


    }

    public boolean isSampleDetected(){
        return getDistance()<=SAMPLE_DETECTION_RANGE;
    }

    public SampleColor sampleColor(){
        if(getBlue()>getGreen() && getBlue()>getRed()){
            return SampleColor.BLUE;
        }
        else if(getRed()/getGreen() < RED_TO_GREEN_YELLOW_RATIO){
            return SampleColor.YELLOW;
        }
        else{
            return SampleColor.RED;
        }
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


}
