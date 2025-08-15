package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.components.GlobalData.OpModeType;

public class Arm {

    /* =========================
       CONFIGURATION CONSTANTS
       ========================= */

    // Gear ratios
    private static final double OUTER_GEAR_RATIO         = 1;
    private static final double INNER_MOTOR_GEAR_RATIO   = 250047.0 / 4913;
    private static final double TICKS_PER_INNER_ROTATION = 28;
    private static final double TICKS_PER_DEGREE = OUTER_GEAR_RATIO
            * INNER_MOTOR_GEAR_RATIO
            * TICKS_PER_INNER_ROTATION / 360.0;

    // Preset positions (degrees)
    public static final double DEFAULT           = 84;
    public static final double PREPARE_SPECIMEN  = 210;
    public static final double SCORE_SPECIMEN    = 148;
    public static final double PREPARE_SUBMERSIBLE = 275;
    public static final double COLLECT           = 300;

    // Soft limits
    private static final double MIN_SOFT_LIMIT = DEFAULT;
    private static final double MAX_SOFT_LIMIT = COLLECT;

    // Control tolerances
    private static final double NO_POWER_TOLERANCE     = 10;
    private static final double ANGLES_EQUAL_TOLERANCE = 5;
    private static final double MANUAL_POWER_MODIFIER  = 0.4;
    private static final double NO_MOVEMENT_TOLERANCE = 0.05;

    // PID + Feedforward constants
    private static double p = 0.028, i = 0, d = 0.002;
    private static double f = 0.13;

    /* =========================
       HARDWARE
       ========================= */
    private final DcMotor leftMotor;
    private final DcMotor rightMotor;
    private final OpModeType opModeType;

    /* =========================
       CONTROL STATE
       ========================= */
    private enum ControlMode { AUTO, MANUAL }
    private ControlMode controlMode;

    private PIDController pidController;
    private double power; //Todo
    private double deltaAngle;
    private double angle;
    private double prevAngle;
    private double targetAngle;

    /* =========================
       CONSTRUCTOR
       ========================= */
    public Arm(DcMotor leftMotor, DcMotor rightMotor, OpModeType opModeType) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.opModeType = opModeType;

        initData();
        initMotors();
    }

    /* =========================
       INITIALIZATION
       ========================= */
    private void initData() {
        angle       = DEFAULT;
        deltaAngle  = 0;
        targetAngle = DEFAULT;
        power       = 0;
        controlMode = ControlMode.AUTO;

        pidController = new PIDController(p, i, d);
    }

    private void initMotors() {
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        if (opModeType == OpModeType.AUTONOMOUS) {
            setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /* =========================
       EXTERNAL COMMANDS
       ========================= */

    public ControlMode controlModeData; //Todo
    /** Main operation loop — should be called every cycle */
    public void operate() {
        determinePower();
        setMotorsPower(power);

        controlModeData = controlMode; //Todo

        update();
    }

    /** Manual control */
    public void operateManually(double manualPower) {
        controlMode = ControlMode.MANUAL;
        power = manualPower * MANUAL_POWER_MODIFIER + calculateFF(targetAngle);
    }

    /** Set desired arm angle (AUTO mode) */
    public void setTargetAngle(double target) {
        targetAngle = target;
        controlMode = ControlMode.AUTO;
    }

    /* =========================
       CONTROL LOGIC
       ========================= */

    private void determinePower() {
        if (controlMode == ControlMode.AUTO) {
            autoControl();
        }
        limit();
    }

    private void autoControl() {
        pidController.setPID(p, i, d);
        double pid = pidController.calculate(angle, targetAngle);
        double ff = calculateFF(targetAngle);

        power = isPowerReleaseRequired() ? 0 : pid + ff;
    }
    //Todo
    public boolean isPowerReleaseRequired() {
        boolean atMax = isAngleEqual(targetAngle, MAX_SOFT_LIMIT, ANGLES_EQUAL_TOLERANCE) &&
                isAngleEqual(angle, MAX_SOFT_LIMIT, NO_POWER_TOLERANCE);

        boolean atMin = isAngleEqual(targetAngle, MIN_SOFT_LIMIT, ANGLES_EQUAL_TOLERANCE) &&
                isAngleEqual(angle, MIN_SOFT_LIMIT, NO_POWER_TOLERANCE);

        return atMax || atMin;
    }

    private void limit() {
        if (angle <= MIN_SOFT_LIMIT) {
            power = Math.max(power, 0);
        } else if (angle >= MAX_SOFT_LIMIT) {
            power = Math.min(power, 0);
        }
        power = Math.max(-1, Math.min(1, power));
    }

    private void update() {
        updateData();
        controlMode = ControlMode.AUTO;
    }

    private void updateData() {
        prevAngle = angle;
        deltaAngle = ticksToDegrees(leftMotor.getCurrentPosition());
        angle = DEFAULT + deltaAngle;

        if (controlMode == ControlMode.MANUAL) {
            targetAngle = angle;
        }
    }

    /* =========================
       UTILITY
       ========================= */
    private double calculateFF(double angle) {
        return Math.cos(Math.toRadians(angle - 90)) * f;
    }

    private boolean isAngleEqual(double a1, double a2, double tolerance) {
        return Math.abs(a1 - a2) <= tolerance;
    }

    private void setRunMode(DcMotor.RunMode runMode) {
        leftMotor.setMode(runMode);
        rightMotor.setMode(runMode);
    }

    private void setMotorsPower(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    private double ticksToDegrees(int ticks) {
        return ticks / TICKS_PER_DEGREE;
    }


        /* =========================
            EXTERNAL DATA ACESS
       ========================= */

    public boolean arrivedTargetAngle(){
        return isAngleEqual(angle, targetAngle, ANGLES_EQUAL_TOLERANCE);
    }

    public boolean armNotMoving() {
        return Math.abs(prevAngle-angle) <= NO_MOVEMENT_TOLERANCE;
    }
}
