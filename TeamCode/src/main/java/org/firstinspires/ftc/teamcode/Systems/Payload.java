package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.Components.ColorSensor;
import org.firstinspires.ftc.teamcode.Components.EndEffector;
import org.firstinspires.ftc.teamcode.Components.GlobalData;
import org.firstinspires.ftc.teamcode.Components.Leds;

public class Payload {
    private Leds leds;
    private EndEffector endEffector;
    private ColorSensor colorSensor;
    private enum ControlMode {INTAKE, UNLOAD, HOLD}
    private ControlMode controlMode;
    private GlobalData.Alliance alliance;
    public Payload(CRServo leftServo, CRServo rightServo, NormalizedColorSensor colorSensor, GlobalData.Alliance alliance, RevBlinkinLedDriver leds) {
        this.endEffector = new EndEffector(leftServo, rightServo);
        this.colorSensor = new ColorSensor(colorSensor);
        this.leds = new Leds(leds);
        this.alliance = alliance;
        controlMode = ControlMode.HOLD;
    }
    public void operate() {
        colorSensor.operate();
        determineControlMode();
        connectModeToBehaviour();

        update();
    }
    private void determineControlMode() {
        if (colorSensor.isSampleDetected() && isOpposingAllianceSampleIn()) {
            controlMode = ControlMode.UNLOAD;
        }
        else if (colorSensor.isSampleDetected() && !isOpposingAllianceSampleIn()) {
            controlMode = ControlMode.HOLD;
        }
    }
    private void connectModeToBehaviour() {
        if (controlMode == ControlMode.INTAKE) endEffector.intake();
        else if (controlMode == ControlMode.UNLOAD) endEffector.unload();
        else endEffector.hold();
    }
    private void update() {
        controlMode = ControlMode.HOLD;
    }
    private boolean isOpposingAllianceSampleIn() {
        if (alliance == GlobalData.Alliance.BLUE && colorSensor.getSampleColor() == GlobalData.SampleColor.RED) {
            return true;
        }
        else if (alliance == GlobalData.Alliance.RED && colorSensor.getSampleColor() == GlobalData.SampleColor.RED) {
            return true;
        }
        else return false;
    }
    public void intake() {
        controlMode = ControlMode.INTAKE;
    }
    public void unload() {
        controlMode = ControlMode.UNLOAD;
    }
    private void operateLeds() {
        if (colorSensor.isSampleDetected()) {
            leds.displaySampleColor(colorSensor.getSampleColor());
        } else {
            leds.turnOff();
        }
    }


}
