package org.firstinspires.ftc.teamcode.Programs.Teleop;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
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
import org.firstinspires.ftc.teamcode.systems.Drivetrain;
import org.firstinspires.ftc.teamcode.systems.Payload;

public class TeleopController {

    /* =========================
       COMPONENTS & INPUT
       ========================= */
    private final Arm arm;
    private final Drivetrain drivetrain;
    private final Payload payload;

    private final GamepadEx gamepadEx1;
    private final GamepadEx gamepadEx2;

    private final Telemetry telemetry;

    /* =========================
       CLASS VARIABLES
   ========================= */

    private long lastLoopTime = System.nanoTime();
    private double loopHz = 0;


    /* =========================
       CONFIGURATION CONSTANTS
       ========================= */
    private static final double TRIGGER_DOWN_TOLERANCE = 0.05;

    /* =========================
       CONSTRUCTOR
       ========================= */
    public TeleopController(OpMode opMode, GlobalData.Alliance alliance, double headingOffset) {

        HardwareMap hardwareMap = opMode.hardwareMap;

        this.telemetry = opMode.telemetry;

        // Initialize the Arm component with the two arm motors
        this.arm = new Arm(
                hardwareMap.get(DcMotorEx.class, "leftArm"),
                hardwareMap.get(DcMotorEx.class, "rightArm"),
                GlobalData.OpModeType.TELEOP
        );

        // Initialize the Drivetrain with OTOS and 4 drive motors
        this.drivetrain = new Drivetrain(
                hardwareMap.get(SparkFunOTOS.class, "otos"), Math.toRadians(0),
                hardwareMap.get(DcMotorEx.class, "leftFront"),
                hardwareMap.get(DcMotorEx.class, "leftBack"),
                hardwareMap.get(DcMotorEx.class, "rightFront"),
                hardwareMap.get(DcMotorEx.class, "rightBack")
        );

        // Initialize Payload with LEDs, color sensor, servos, and alliance color
        this.payload = new Payload(
                hardwareMap.get(RevBlinkinLedDriver.class, "leds"),
                hardwareMap.get(NormalizedColorSensor.class, "colorSensor"),
                hardwareMap.get(CRServo.class, "rightServo"),
                hardwareMap.get(CRServo.class, "leftServo"),
                alliance
        );

        this.gamepadEx1 = new GamepadEx(opMode.gamepad1);
        this.gamepadEx2 = new GamepadEx(opMode.gamepad2);
    }

    /* =========================
       MAIN OPERATION LOOP
       ========================= */
    public void operate() {
        operateArm();
        operateDrivetrain();
        operatePayload();
        operateTelemetry();
        operateGamepads();
    }

    /* =========================
       ARM CONTROL LOGIC
       ========================= */
    private void operateArm() {
        // Manual arm control if either trigger is pressed beyond tolerance

        double manualPower = gamepadEx2.gamepad.right_trigger - gamepadEx2.gamepad.left_trigger;

        if (Math.abs(manualPower) > TRIGGER_DOWN_TOLERANCE) {
            arm.operateManually(manualPower);

        } else {
            // Otherwise, check for preset arm positions from gamepad2 buttons
            if (gamepadEx2.wasJustPressed(GamepadKeys.Button.X)) {
                arm.setTargetAngle(Arm.PREPARE_SPECIMEN);
            } else if (gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                arm.setTargetAngle(Arm.SCORE_SPECIMEN);
            } else if (gamepadEx2.wasJustPressed(GamepadKeys.Button.A)) {
                arm.setTargetAngle(Arm.DEFAULT);
            } else if (gamepadEx2.wasJustPressed(GamepadKeys.Button.Y)) {
                arm.setTargetAngle(Arm.COLLECT);
            } else if (gamepadEx2.wasJustPressed(GamepadKeys.Button.B)) {
                arm.setTargetAngle(Arm.PREPARE_SUBMERSIBLE);
            }
        }

        arm.operate();
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

    /* =========================
       PAYLOAD CONTROL LOGIC
       ========================= */
    private void operatePayload() {
        // Control payload intake/unload with bumpers on gamepad2
        if (gamepadEx2.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
            payload.unload();
        } else if (gamepadEx2.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            payload.intake();
        }


        payload.operate();
    }

    @SuppressLint("DefaultLocale")
    private String getLoopsPerSecond() {
        long now = System.nanoTime();
        double loopTimeSec = (now - lastLoopTime) / 1e9;
        lastLoopTime = now;

        if (loopTimeSec > 0) {
            loopHz = 1.0 / loopTimeSec;
        }
        return String.format("%.1f", loopHz);
    }


    private void operateTelemetry(){
        telemetry.addData("heading", drivetrain.getHeading());
        telemetry.update();
    }

    private void operateGamepads(){
        gamepadEx1.readButtons();
        gamepadEx2.readButtons();
    }




}
