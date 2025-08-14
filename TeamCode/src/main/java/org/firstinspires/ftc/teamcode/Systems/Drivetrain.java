package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.DriveComponent;
import org.firstinspires.ftc.teamcode.Components.Otos;


public class Drivetrain {
    private DriveComponent driveComponent;
    private Otos otos;
    public Drivetrain(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, SparkFunOTOS otos, double headingOffset) {
        this.driveComponent = new DriveComponent(leftFront, rightFront, leftBack, rightBack);
        this.otos = new Otos(otos, headingOffset);
    }
    private void determineSpeed(double spdX, double spdY, double spdT) {
        driveComponent.determineSpeed(spdX, spdY, spdT);
        driveComponent.adjustSpeed(otos.getHeading());
    }
    public void operate(double spdX, double spdY, double spdT, boolean changeSpdModeRequested, boolean resetHeadingRequested) {
        if (changeSpdModeRequested) driveComponent.changeSpdMode();
        if (resetHeadingRequested) otos.resetHeading();
        determineSpeed(spdX, spdY, spdT);
        driveComponent.translateSpeedToPower();
    }
}
