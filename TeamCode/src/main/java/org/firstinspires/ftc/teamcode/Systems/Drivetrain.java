package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.DriveComponent;
import org.firstinspires.ftc.teamcode.Components.MecanumDriveComponent;
import org.firstinspires.ftc.teamcode.Components.Otos;

public class Drivetrain {

    /* =========================
       COMPONENTS
       ========================= */
    private final DriveComponent driveComponent;
    private final Otos otos;

    /* =========================
       CONSTRUCTOR
       ========================= */
    public Drivetrain(SparkFunOTOS otosSensor, double headingOffset,
                      DcMotor leftFront, DcMotor leftBack,
                      DcMotor rightFront, DcMotor rightBack) {
        this.driveComponent = new DriveComponent(leftFront, rightFront, leftBack, rightBack);
        this.otos = new Otos(otosSensor, headingOffset);
    }

    /* =========================
       INTERNAL SPEED CALCULATION
       ========================= */
    private void determineSpd(double spdX, double spdY, double spdTurn) {
        // Set base speeds
        driveComponent.determineSpeed(spdX, spdY, spdTurn);

        // Adjust speeds based on current heading from OTOS sensor (field-centric control)
        driveComponent.adjustSpeed(otos.getHeading());
    }

    /* =========================
       MAIN OPERATION
       ========================= */
    public void operate() {
        driveComponent.translateSpeedToPower();
    }

    /* =========================
       EXTERNAL INTERFACE
       ========================= */
    public void moveManually(double spdX, double spdY, double spdT) {
        determineSpd(spdX, spdY, spdT);
    }

    public void changeSpdMode() {
        driveComponent.toggleSpeedMode();
    }

    public void resetHeading() {
        otos.resetHeading();
    }

    public double getHeading(){
        return otos.getHeading();
    }

    public double getSpdX(){
        return driveComponent.getSpdX();
    }

    public double getSpdY(){
        return driveComponent.getSpdY();
    }

    public double getSpdTurn(){
        return driveComponent.getSpdTurn();
    }


}