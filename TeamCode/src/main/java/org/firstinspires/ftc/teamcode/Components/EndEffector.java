package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class EndEffector  {

    /* =========================
       CONFIGURATION CONSTANTS
       ========================= */
    private static final double POWER_INTAKE = 1.0;
    private static final double POWER_UNLOAD = -1.0;
    private static final double POWER_HOLD   = 0.0;

    /* =========================
       HARDWARE REFERENCES
       ========================= */
    private final CRServo servoRight;
    private final CRServo servoLeft;

    /* =========================
       CONSTRUCTOR
       ========================= */
    public EndEffector(CRServo right, CRServo left) {
        this.servoRight = right;
        this.servoLeft  = left;
        this.servoRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /* =========================
       PUBLIC CONTROL METHODS
       ========================= */
    public void intake() {
        setServoPower(POWER_INTAKE);
    }

    public void unload() {
        setServoPower(POWER_UNLOAD);
    }

    public void hold() {
        setServoPower(POWER_HOLD);
    }

    /* =========================
       PRIVATE HELPERS
       ========================= */
    private void setServoPower(double power) {
        servoLeft.setPower(power);
        servoRight.setPower(power);
    }
}
