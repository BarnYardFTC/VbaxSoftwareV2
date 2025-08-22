package org.firstinspires.ftc.teamcode.Programs.Teleop;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Components.Arm;
import org.firstinspires.ftc.teamcode.Components.GlobalData;
import org.firstinspires.ftc.teamcode.Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Systems.Payload;

public class TeleopController {

    /* =========================
       COMPONENTS & INPUT
       ========================= */
    private final Drivetrain drivetrain;
    private final Arm arm;

    private final GamepadEx gamepadEx1;
    private final GamepadEx gamepadEx2;

    private final Telemetry telemetry;


    /* =========================
       CONFIGURATION CONSTANTS
       ========================= */
    private static final double TRIGGER_DOWN_TOLERANCE = 0.05;

    /* =========================
       CONSTRUCTOR
       ========================= */
    public TeleopController(OpMode opMode, double headingOffset) {

        HardwareMap hardwareMap = opMode.hardwareMap;

        this.telemetry = opMode.telemetry;


        // Initialize the Drivetrain with OTOS and 4 drive motors
        this.drivetrain = new Drivetrain(
                hardwareMap.get(SparkFunOTOS.class, "otos"), headingOffset,
                hardwareMap.get(DcMotorEx.class, "leftFront"),
                hardwareMap.get(DcMotorEx.class, "leftBack"),
                hardwareMap.get(DcMotorEx.class, "rightFront"),
                hardwareMap.get(DcMotorEx.class, "rightBack")
        );


        this.arm = new Arm(
                hardwareMap.get(DcMotorEx.class , "rightArm"),
                hardwareMap.get(DcMotorEx.class,"leftArm"),
                GlobalData.OpmodeType.TELEOP
        );



        this.gamepadEx1 = new GamepadEx(opMode.gamepad1);
        this.gamepadEx2 = new GamepadEx(opMode.gamepad2);
    }

    /* =========================
       MAIN OPERATION LOOP
       ========================= */
    public void operate() {
        operateDrivetrain();
        operateTelemetry();
        operateGamepads();
        operateArm();
    }


    /* =========================
       DRIVETRAIN CONTROL LOGIC
       ========================= */

    private void operateDrivetrain() {
        // Toggle speed mode on button A press (gamepad1)
        if (gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) {
            drivetrain.changeSpdMode();
        }

        // Reset heading on button X press (gamepad1)
        if (gamepadEx1.wasJustPressed(GamepadKeys.Button.X)) {
            drivetrain.resetHeading();
        }

        // Drive robot manually using left stick (translation) and right stick (rotation)
        drivetrain.moveManually(
                gamepadEx1.getLeftX(),
                gamepadEx1.getLeftY(),
                gamepadEx1.getRightX()
        );

        drivetrain.operate();
    }

    private void operateArm(){
        double manualPower = gamepadEx2.gamepad.right_trigger - gamepadEx2.gamepad.left_trigger;
        if (Math.abs(manualPower) > 0){
            arm.operateManually(manualPower);
        }
        else if (gamepadEx2.wasJustPressed(GamepadKeys.Button.A)) {
            arm.setTargetAngle(Arm.DEFAULT);
        }
        else if (gamepadEx2.wasJustPressed(GamepadKeys.Button.B)) {
            arm.setTargetAngle(Arm.PREP_SUB);
        }
        else if (gamepadEx2.wasJustPressed(GamepadKeys.Button.X)) {
            arm.setTargetAngle(Arm.SCORE_SPECIMEN);
        }
        else if (gamepadEx2.wasJustPressed(GamepadKeys.Button.Y)) {
            arm.setTargetAngle(Arm.COLLECT);
        }
        else if (gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            arm.setTargetAngle(Arm.PREP_SPECIMEN);
        }

        arm.operate();
    }


    private void operateTelemetry(){
        telemetry.addData("angle" ,  arm.getAngle());
        telemetry.addData("targetAngle" , arm.getTargetAngle());
        telemetry.addData("controlMode" , arm.getControlMode());
        telemetry.addData("isPowerReleaseRequired",arm.isPowerReleaseRequired());
        telemetry.addData("power" , arm.getPower());
        telemetry.addData("leftPower", arm.getLeftPower());
        telemetry.addData("rightPower", arm.getRightPower());
        telemetry.update();
    }

    private void operateGamepads(){
        gamepadEx1.readButtons();
        gamepadEx2.readButtons();
    }
}