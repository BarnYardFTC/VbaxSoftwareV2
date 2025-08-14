package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DriveComponent {
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private double speedX, speedY, speedTurn;
    private double speedModifier;
    private final double SLOW_SPEED = 0.3;
    private final double FAST_SPEED = 1.0;

    public DriveComponent(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
        this.leftFront = leftFront;
        this.rightFront =rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
        initMotor(DcMotorSimple.Direction.REVERSE, leftFront);
        initMotor(DcMotorSimple.Direction.FORWARD, rightFront);
        initMotor(DcMotorSimple.Direction.REVERSE, leftBack);
        initMotor(DcMotorSimple.Direction.FORWARD, rightBack);
        initData();
    }

    private void initMotor(DcMotorSimple.Direction direction, DcMotor motor){
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private void initData() {
        speedX = 0;
        speedY = 0;
        speedTurn = 0;
        activateFastMode();
    }
    public void activateSlowMode() {
        speedModifier = SLOW_SPEED;
    }
    public void activateFastMode() {
        speedModifier = FAST_SPEED;
    }
    public void changeSpdMode() {
        if (speedModifier == SLOW_SPEED) activateFastMode();
        else activateSlowMode();
    }

    public void determineSpeed(double spdX, double spdY, double spdT) {
        speedX = spdX;
        speedY = spdY;
        speedTurn = spdT;
    }

    public void translateSpeedToPower() {
        double lf = speedY + speedX + speedTurn;
        double lb = speedY - speedX + speedTurn;
        double rf = speedY - speedX - speedTurn;
        double rb = speedY + speedX - speedTurn;

        double maxPower = Math.max(Math.max(Math.abs(lf), Math.abs(lb)),
                Math.max(Math.abs(rf), Math.abs(rb)));

        if (maxPower > 1.0) {
            lf /= maxPower;
            lb /= maxPower;
            rf /= maxPower;
            rb /= maxPower;
        }

        lf *= speedModifier;
        lb *= speedModifier;
        rf *= speedModifier;
        rb *= speedModifier;

        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightFront.setPower(rf);
        rightBack.setPower(rb);
    }

    public void adjustSpeed(double heading) {
        double adjustedY,
                adjustedX;
        adjustedX = speedX * Math.cos(heading) - speedY * Math.sin(heading);
        adjustedY = speedX * Math.sin(heading) + speedY * Math.cos(heading);
        speedX = adjustedX;
        speedY = adjustedY;
    }
}
