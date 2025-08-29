package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.Components.ColorSensor;
import org.firstinspires.ftc.teamcode.Components.EndEffector;
import org.firstinspires.ftc.teamcode.Components.GlobalData;

public class Payload {
    public EndEffector endEffector;
    public ColorSensor colorSensor;

    private enum ControlMode {UNLOAD, INTAKE, HOLD}

    ;
    public ControlMode controlMode;
    public GlobalData.Alliance alliance;

    public Payload(NormalizedColorSensor colorSensor, CRServo rightServo,
                   CRServo leftServo, GlobalData.Alliance alliance) {
        this.colorSensor = new ColorSensor(colorSensor);
        this.endEffector = new EndEffector(rightServo, leftServo);
        this.alliance = alliance;
        this.controlMode = ControlMode.HOLD;

    }

    public void determineControlMode(){
        if(isOpposingSampleIn()){
            controlMode = ControlMode.UNLOAD;
        }
        if(colorSensor.isSampleDetected() && controlMode == ControlMode.INTAKE){
            controlMode = ControlMode.HOLD;
        }

    }

    public void unload() {
        controlMode = ControlMode.UNLOAD;
    }

    public void intake() {
        controlMode = ControlMode.INTAKE;

    }

    private boolean isOpposingSampleIn() {
        if (colorSensor.isSampleDetected()) {
            if (alliance == GlobalData.Alliance.RED && colorSensor.sampleColor() == GlobalData.SampleColor.BLUE) {
                return true;
            } else if (alliance == GlobalData.Alliance.BLUE && colorSensor.sampleColor() == GlobalData.SampleColor.RED) {
                return true;
            }

        }
        return false;

    }


    public void operate(){
        determineControlMode();
        connectModetoBehaviour();
        update();
    }

    public void connectModetoBehaviour(){
        if(controlMode == ControlMode.HOLD){
            endEffector.hold();
        }
        else if (controlMode == ControlMode.INTAKE){
            endEffector.intake();
        }

        else{endEffector.unload();}
    }

    public void update(){
        controlMode = ControlMode.HOLD;
    }
}









