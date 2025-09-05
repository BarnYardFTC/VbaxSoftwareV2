package org.firstinspires.ftc.teamcode.systems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;

public class PIDDrive extends Drivetrain{
    private PIDController pidX;
    private PIDController pidY;
    private PIDController pidA;


    private double targetX, targetY, targetA;

    public PIDDrive(SparkFunOTOS otosSensor, double headingOffset,
                    DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack,
                    double targetX, double targetY, double targetA) {
        super(otosSensor, headingOffset, leftFront, leftBack, rightFront, rightBack);
        setTarget(targetX, targetY, targetA);
        pidX = new PIDController(0, 0, 0);
        pidY = new PIDController(0, 0, 0);
        pidA = new PIDController(0, 0, 0);
    }

    private void setTarget(double x, double y, double a){
        this.targetX = x;
        this.targetY = y;
        this.targetA = a;
    }

    private void moveToTarget(double currX, double currY, double currA){

        double deltaX = targetX - currX;
        double deltaY = targetY - currY;
        double deltaA = targetA - currA;

        double vx = pidX.calculate(deltaX);
        double vy = pidX.calculate(deltaY);
        double va = pidX.calculate(deltaA);

        moveManually(vx, vy, va);
    }
}
