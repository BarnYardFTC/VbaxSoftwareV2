package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.ColorSensorEx;
import org.firstinspires.ftc.teamcode.Components.EndEffector;
import org.firstinspires.ftc.teamcode.Components.GlobalData.Alliance;
import org.firstinspires.ftc.teamcode.Components.GlobalData.SampleColor;
import org.firstinspires.ftc.teamcode.Components.Leds;

@Config
public class   Payload {

    /* =========================
       CONFIGURATION CONSTANTS
       ========================= */
    public static int INTAKE_TIME = 300; // ms delay after sample detected before switching HOLD

    /* =========================
       ENUMS
       ========================= */
    private enum ControlMode { INTAKE, TIMED_INTAKE, UNLOAD, HOLD }

    /* =========================
       FIELDS: COMPONENTS
       ========================= */
    private final Leds leds;
    private final EndEffector endEffector;
    private final ColorSensorEx colorSensor;
    private final Alliance alliance;

    /* =========================
       FIELDS: CONTROL & STATE
       ========================= */
    private ControlMode controlMode;
    private ControlMode reqControlMode;

    private final ElapsedTime intakeTimer = new ElapsedTime();
    public boolean timerStarted;
    public boolean sampleIn;

    /* =========================
       CONSTRUCTOR
       ========================= */
    public Payload(RevBlinkinLedDriver leds, NormalizedColorSensor colorSensor,
                   CRServo servoR, CRServo servoL, Alliance alliance) {
        this.leds = new Leds(leds);
        this.colorSensor = new ColorSensorEx(colorSensor);
        this.endEffector = new EndEffector(servoR, servoL);
        this.alliance = alliance;
        this.controlMode = ControlMode.HOLD;
        this.reqControlMode = ControlMode.HOLD;
        timerStarted = false;
        sampleIn = false;
    }

    /* =========================
       MAIN OPERATION METHOD
       ========================= */
    public void operate() {
        colorSensor.operate();
        determineControlMode();

        if (controlMode == ControlMode.INTAKE) endEffector.intake();
        else if (controlMode == ControlMode.TIMED_INTAKE) handleTimedIntake();
        else if (controlMode == ControlMode.UNLOAD) endEffector.unload();
        else endEffector.hold();

        operateLeds();
        update();
    }

    /* =========================
       CONTROL MODE DECISION METHODS
       ========================= */
    private void determineControlMode() {
        if (isOpposingSampleIn()) controlMode = ControlMode.UNLOAD;
        else if (reqControlMode == ControlMode.INTAKE){
            if (sampleIn) controlMode = ControlMode.HOLD;
            else if (isSampleDetected()) controlMode = ControlMode.TIMED_INTAKE;
            else controlMode = ControlMode.INTAKE;
        }
        else if (controlMode == ControlMode.TIMED_INTAKE && reqControlMode == ControlMode.UNLOAD)
            interruptIntakeAndUnload();
        else if (controlMode == ControlMode.TIMED_INTAKE && !isSampleDetected()) controlMode = reqControlMode;
        else controlMode = reqControlMode;
    }

    private void handleTimedIntake() {
        if (!timerStarted) {
            intakeTimer.reset();
            timerStarted = true;
        }

        if (intakeTimer.milliseconds() >= INTAKE_TIME) {
            controlMode = ControlMode.HOLD;
            resetIntakeTimer();
            sampleIn = true;
        }
        else {
            endEffector.intake();
        }
    }

    private void interruptIntakeAndUnload() {
        controlMode = ControlMode.UNLOAD;
        resetIntakeTimer();
    }

    private void resetIntakeTimer() {
        timerStarted = false;
        intakeTimer.reset();
    }

    /* =========================
       SENSOR & STATE HELPERS
       ========================= */
    public boolean isOpposingSampleIn() {
        boolean isOpposingSample = (alliance == Alliance.RED && colorSensor.getSampleColor() == SampleColor.BLUE)
                || (alliance == Alliance.BLUE && colorSensor.getSampleColor() == SampleColor.RED);
        return isOpposingSample && colorSensor.isSampleDetected();
    }

    private void operateLeds() {
        if (colorSensor.isSampleDetected()) {
            leds.displaySampleColor(colorSensor.getSampleColor());
        } else {
            leds.turnOff();
        }
    }

    /* =========================
       EXTERNAL COMMANDS
       ========================= */
    public void intake() {
        reqControlMode = ControlMode.INTAKE;
    }

    public void unload() {
        reqControlMode = ControlMode.UNLOAD;
    }

    public boolean isSampleDetected() {
        return colorSensor.isSampleDetected();
    }

    /* =========================
       STATE UPDATES
       ========================= */
    private void update(){
        if (controlMode == ControlMode.UNLOAD || !isSampleDetected()) sampleIn = false;
        if (controlMode == ControlMode.TIMED_INTAKE) reqControlMode = ControlMode.INTAKE;
        else reqControlMode = ControlMode.HOLD;
    }

}
