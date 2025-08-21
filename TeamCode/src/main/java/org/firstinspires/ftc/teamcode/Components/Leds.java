package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.teamcode.Components.GlobalData.SampleColor;

public class Leds {

    /* =========================
       HARDWARE REFERENCES
       ========================= */
    private final RevBlinkinLedDriver ledDriver;

    /* =========================
       STATE
       ========================= */
    private RevBlinkinLedDriver.BlinkinPattern currentPattern = null;

    /* =========================
       CONSTRUCTOR
       ========================= */
    public Leds(RevBlinkinLedDriver ledDriver) {
        this.ledDriver = ledDriver;
    }

    /* =========================
       PUBLIC METHODS
       ========================= */
    public void displaySampleColor(SampleColor sampleColor) {
        switch (sampleColor) {
            case RED:
                displayRed();
                break;
            case BLUE:
                displayBlue();
                break;
            case YELLOW:
                displayYellow();
                break;
            default:
                turnOff();
                break;
        }
    }

    public void turnOff() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    /* =========================
       PRIVATE HELPERS
       ========================= */
    private void setPattern(RevBlinkinLedDriver.BlinkinPattern newPattern) {
        if (newPattern != currentPattern) {
            ledDriver.setPattern(newPattern);
            currentPattern = newPattern;
        }
    }

    private void displayBlue() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }

    private void displayRed() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
    }

    private void displayYellow() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
    }
}
