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

    private final double ANGLE_TOLERANCE = 5;
    private final double ZERO_POWER_TOLERANCE = 15;
    private final double MANUAL_MODIFIER = 0.4;

    public static double p = 0.028, d = 0.0018;
    public static double f = 0.13;
    private PIDController pidController;

    public enum ControlMode {AUTO_CONTROL, MANUAL_CONTROL}
    private ControlMode controlMode;

    private GlobalData.OpmodeType opmodeType;

    private double power;
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
        targetAngle = angle;
        power = 0;

        controlMode = ControlMode.AUTO_CONTROL;
        pidController = new PIDController(p,0,d);
    }

    public void operate(){
        determinePower();
        setPower();
        update();
    }

    private void determinePower(){
        if (controlMode == ControlMode.AUTO_CONTROL){
            if (!isPowerReleaseRequired()) auto();
            else power = 0;
        }
        limitPower();
    }

    private void auto(){
        pidController.setPID(p,0,d);
        double pid = pidController.calculate(angle, targetAngle);
        power = pid + calculateFF();
    }

    public void operateManually(double power){
        controlMode = ControlMode.MANUAL_CONTROL;
        this.power = power * MANUAL_MODIFIER+calculateFF();
    }


    private void setPower(){
        rightArm.setPower(power);
        leftArm.setPower(power);
    }

    private void update(){
        updateData();
        if (controlMode == ControlMode.MANUAL_CONTROL) targetAngle = angle;
        controlMode = ControlMode.AUTO_CONTROL;
    }

    private void limitPower(){
        if(isAngleEqual(angle,MINIMUM_SOFT_LIMIT,ANGLE_TOLERANCE)) {
            power = Math.max(0, power);
        }
        else if(isAngleEqual(angle, MAXIMUM_SOFT_LIMIT, ANGLE_TOLERANCE)){
            power = Math.min(0,power);
        }
    }

    private void updateData(){
        double deltaAngle = encodersToDegrees(leftArm.getCurrentPosition());
        angle = deltaAngle + MINIMUM_SOFT_LIMIT; //or default
    }

    private double calculateFF() {
        return f * Math.cos(Math.toRadians(angle - 90)) ; // you're basically subtracting 90
                                                          // to turn the unit circle 90 degrees
    }

    public boolean isPowerReleaseRequired(){
        if(isAngleEqual(angle,targetAngle,ZERO_POWER_TOLERANCE)&&
         (targetAngle == MAXIMUM_SOFT_LIMIT || targetAngle == MINIMUM_SOFT_LIMIT)){
            return true;
        }
        else{
            return false;
        }
    }

    private boolean isAngleEqual(double a1, double a2, double tolerance){// a1 = 50, a2 = 50.65, tolerance = 1
        if(Math.abs(a2-a1)<=tolerance){
            return true;
        }
        else{return false;}
    }

    private double encodersToDegrees(double encoders){
        return encoders / TICKS_PER_DEGREE;
    }



    public double getAngle(){
        return angle;
    }
    public double getPower(){
        return power;
    }
    public double getLeftPower(){
        return leftArm.getPower();
    }
    public double getRightPower(){
        return rightArm.getPower();
    }

    public void setTargetAngle(double targetAngle){
        this.targetAngle = targetAngle;
    }

    public double getTargetAngle(){
        return targetAngle;
    }

    public boolean hasArrivedTarget(){
        return isAngleEqual(angle, targetAngle, ANGLE_TOLERANCE);
    }

    public ControlMode getControlMode(){
        return controlMode;
    }
}
