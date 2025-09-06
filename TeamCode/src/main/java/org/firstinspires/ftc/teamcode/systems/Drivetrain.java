package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.Limelight;
import org.firstinspires.ftc.teamcode.components.MecanumDriveComponent;
import org.firstinspires.ftc.teamcode.components.Otos;


@Config
public class Drivetrain {

    /* =========================
       COMPONENTS
       ========================= */
    private MecanumDriveComponent driveComponent;
    private Otos otos;

    private Limelight limelight;
    private PIDController pidControllerX, pidControllerY, pidControllerT;
    public static double pX = 1.5, dX = 0, pY = 1.5, dY = 0, pT = 0.05, dT = 0;

    private double Y_DISTANCE = 0.22;


    /* =========================
       CONSTRUCTOR
       ========================= */
    public Drivetrain(SparkFunOTOS otosSensor, double headingOffset,
                      DcMotor leftFront, DcMotor leftBack,
                      DcMotor rightFront, DcMotor rightBack) {
        this.driveComponent = new MecanumDriveComponent(leftFront, rightFront, leftBack, rightBack);
        this.otos = new Otos(otosSensor, headingOffset);
    }

    public Drivetrain(Limelight3A limelight3A,
                      DcMotor leftFront, DcMotor leftBack,
                      DcMotor rightFront, DcMotor rightBack) {
        this.driveComponent = new MecanumDriveComponent(leftFront, rightFront, leftBack, rightBack);

        this.limelight = new Limelight(limelight3A);
        pidControllerX = new PIDController(pX, 0, dX);
        pidControllerY = new PIDController(pY, 0, dY);
        pidControllerT = new PIDController(pT, 0, dT);
    }

    public void start(){
        limelight.start();
    }

    /* =========================
       INTERNAL SPEED CALCULATION
       ========================= */
    private void determineSpd(double spdX, double spdY, double spdTurn) {
        // Set base speeds
        driveComponent.setSpeed(spdX, spdY, spdTurn);

        // Adjust speeds based on current heading from OTOS sensor (field-centric control)
        driveComponent.adjustSpeedForHeading(otos.getHeading());
    }

    double spdX, spdT, spdY, Dx, Dy, Dt;


    private void determineSpdBasedOnD(double Dx, double Dy, double Dt){
        spdX = convertDiffToSpd(Dx, 0, pidControllerX);
        spdY = convertDiffToSpd(Dy, -Y_DISTANCE ,pidControllerY);
        spdT = convertDiffToSpd(Dt, 0, pidControllerT);

        driveComponent.setSpeed(-spdX, spdY, spdT);
        driveComponent.adjustSpeedForHeading(Math.toRadians(-Dt));

        // TODO telemtry - temp
        this.Dt = Dt;
        this.Dy = Dy;
        this.Dx = Dx;
    }

    private double convertDiffToSpd(double diff, double target, PIDController pidController){
        return pidController.calculate(diff, target);
    }

    /* =========================
       MAIN OPERATION
       ========================= */

    public void operateAuto(){
        limelight.operate();
        pidControllerX.setPID(pX, 0, dX);
        pidControllerY.setPID(pY, 0, dY);
        pidControllerT.setPID(pT, 0, dT);

        determineSpdBasedOnD(limelight.getDx(), limelight.getDy(), limelight.getDt());

        driveComponent.translateSpeedToPower();
    }

    public void displayData(Telemetry telemetry){
        telemetry.addData("Dx", Dx);
        telemetry.addData("Dy", Dy);
        telemetry.addData("Dt", Dt);
        telemetry.addData("converted-spdX", spdX);
        telemetry.addData("converted-spdY", spdY);
        telemetry.addData("converted-spdT", spdT);
        telemetry.addData("final-spdX", driveComponent.getSpdX());
        telemetry.addData("final-spdY", driveComponent.getSpdY());
        telemetry.addData("final-spdT", driveComponent.getSpdTurn());

    }

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
