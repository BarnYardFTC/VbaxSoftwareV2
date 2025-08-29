package org.firstinspires.ftc.teamcode.Components;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Arm {
    private DcMotor leftArm, rightArm;

    private final double OUTER_GEAR_RATIO = 1;
    private final double INNER_MOTOR_GEAR_RATIO = 250047.0 / 4913;
    private final double TICKS_PER_INNER_ROTATION = 28;
    private final double TICKS_PER_DEGREE = OUTER_GEAR_RATIO * INNER_MOTOR_GEAR_RATIO * TICKS_PER_INNER_ROTATION / 360;

    private static final double MINIMUM_SOFT_LIMIT = 84;
    private static final double MAXIMUM_SOFT_LIMIT = 305;

    public static final double DEFAULT = MINIMUM_SOFT_LIMIT;
    public static final double PREP_SPECIMEN = 210;
    public static final double SCORE_SPECIMEN = 170;
    public static final double COLLECT = MAXIMUM_SOFT_LIMIT;
    public static final double PREP_SUB = 275;

    private final double ANGLE_TOLERANCE = 3;
    private final double ZERO_POWER_TOLERANCE = 15;
    private final double MANUAL_MODIFIER = 0.4;

    public static double p = 0.028, i = 0, d = 0.0018;
    public static double f = 0.13;
    private PIDController pidController;

    public enum ControlMode {AUTO_CONTROL, MANUAL_CONTROL}
    private ControlMode controlMode;

    private GlobalData.OpmodeType opmodeType;

    private double power;
    private double manualPower;
    private double deltaAngle;
    private double angle;
    private double targetAngle;
    public Arm(DcMotor leftArm, DcMotor rightArm, GlobalData.OpmodeType opmodeType) {
        this.leftArm = leftArm;
        this.rightArm = rightArm;
        this.opmodeType = opmodeType;
        initMotors();
        initData();
    }
    private void initMotors() {
        rightArm.setDirection(DcMotorSimple.Direction.REVERSE);
        leftArm.setDirection(DcMotorSimple.Direction.FORWARD);
        if (opmodeType == GlobalData.OpmodeType.AUTONOMOUS) {
            leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private void initData() {
        angle = DEFAULT;
        deltaAngle = 0;
        targetAngle = angle;
        power = 0;
        manualPower = 0;
        controlMode = ControlMode.AUTO_CONTROL;
        pidController = new PIDController(p,i,d);
    }
    public void operate() {
        determinePower();
        setPower();
        update();
    }
    private void determinePower() {
        if (controlMode == ControlMode.AUTO_CONTROL) {
            auto();
        }
        else {
            manual();
        }
        limitPower();
    }
    private void limitPower() {
        if (angle <= MINIMUM_SOFT_LIMIT) power = Math.max(power, 0);
        else if (angle >= MAXIMUM_SOFT_LIMIT) power = Math.min(power, 0);
        power = Math.max(-1, Math.min(1, power));
    }
    private void setPower() {
        leftArm.setPower(power);
        rightArm.setPower(power);
    }
    private void update() {
        updateData();updateData();
        controlMode = ControlMode.AUTO_CONTROL;
    }
    private void updateData() {
        deltaAngle = ticksToDegrees(leftArm.getCurrentPosition());
        angle = DEFAULT + deltaAngle;
        if (controlMode == ControlMode.MANUAL_CONTROL) targetAngle = angle;
    }
    private void auto() {
        pidController.setPID(p, i, d);
        double pid = pidController.calculate(angle, targetAngle);
        double ff = Math.cos(Math.toRadians(targetAngle - 90)) * f;
        if (isPowerReleaseReq()) power = 0;
        else power = pid + ff;
    }
    private void manual() {
        double ff = Math.cos(Math.toRadians(targetAngle - 90)) * f;
        power = manualPower * MANUAL_MODIFIER + ff;
    }
    public void operateManual(double power) {
        this.manualPower = power;
        controlMode = ControlMode.MANUAL_CONTROL;
    }
    private boolean isPowerReleaseReq() {
        boolean atMin = areAnglesEqual(targetAngle, MINIMUM_SOFT_LIMIT, ANGLE_TOLERANCE) &&
                areAnglesEqual(angle, MINIMUM_SOFT_LIMIT, ZERO_POWER_TOLERANCE);
        boolean atMax = areAnglesEqual(targetAngle, MAXIMUM_SOFT_LIMIT, ANGLE_TOLERANCE) &&
                areAnglesEqual(angle, MAXIMUM_SOFT_LIMIT, ZERO_POWER_TOLERANCE);
        if (atMin || atMax) return true;
        else return false;
    }
    private boolean areAnglesEqual(double angle1, double angle2, double tolerance) {
        if (Math.abs(angle1 - angle2) <= tolerance) return true;
        else return false;
    }
    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
        controlMode = ControlMode.AUTO_CONTROL;
    }
    public boolean hasArmArrived() {
        if (areAnglesEqual(angle, targetAngle, ANGLE_TOLERANCE)) return true;
        else return false;
    }
    private double ticksToDegrees(double ticks) {
        return ticks / TICKS_PER_DEGREE;
    }
}
