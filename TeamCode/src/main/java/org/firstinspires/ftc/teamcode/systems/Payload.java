package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.components.ColorSensor;
import org.firstinspires.ftc.teamcode.components.EndEffector;
import org.firstinspires.ftc.teamcode.components.GlobalData;

public class Payload {
    private EndEffector endEffector;
    private ColorSensor colorSensor;
    private enum ControlMode {INTAKE, UNLOAD, HOLD}
    private ControlMode controlMode;
    private GlobalData.Alliance alliance;
    public Payload(CRServo leftServo, CRServo rightServo, NormalizedColorSensor colorSensor, GlobalData.Alliance alliance) {
        this.endEffector = new EndEffector(leftServo, rightServo);
        this.colorSensor = new ColorSensor(colorSensor);
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
}
