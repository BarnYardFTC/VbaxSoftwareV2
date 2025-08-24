package org.firstinspires.ftc.teamcode.Programs.Teleop;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
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
import org.firstinspires.ftc.teamcode.Components.GlobalData.RobotSystem;
import org.firstinspires.ftc.teamcode.systems.Drivetrain;
import org.firstinspires.ftc.teamcode.systems.Payload;

public class TeleopActionController {

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

    private enum AutomationFlag {
        SCORE_SPECIMEN, COLLECT_SAMPLE, NONE
    }
    private AutomationFlag currentAutomationFlag;

    private Action currentAutomation;

    private boolean automationStopRequested;

    /* =========================
       CONSTRUCTOR
       ========================= */
    public TeleopActionController(OpMode opMode, GlobalData.Alliance alliance, double headingOffset) {

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

        currentAutomationFlag = AutomationFlag.NONE;
        currentAutomation = null;
        automationStopRequested = false;

        this.gamepadEx1 = new GamepadEx(opMode.gamepad1);
        this.gamepadEx2 = new GamepadEx(opMode.gamepad2);
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

    private boolean isSystemAutomating(RobotSystem system){
        if (currentAutomationFlag == AutomationFlag.COLLECT_SAMPLE)
            return system == RobotSystem.PAYLOAD || system == RobotSystem.ARM;

        else if (currentAutomationFlag == AutomationFlag.SCORE_SPECIMEN)
            return system == RobotSystem.PAYLOAD || system == RobotSystem.ARM;

        else return false;
    }

    private boolean isRobotAutomating(){
        return currentAutomationFlag != AutomationFlag.NONE;
    }

    private void chooseAutomation(AutomationFlag automationFlag){
        currentAutomationFlag = automationFlag;
        switch (automationFlag){
            case SCORE_SPECIMEN:
                currentAutomation = scoreSpecimenAutomation();
                break;
            case COLLECT_SAMPLE:
                currentAutomation = collectSampleAutomation();
                break;
            case NONE:
                currentAutomation = null;
                break;
        }
    }



    // ==================== ACTION SECTION ===================


    public Action operate(){
        return new ParallelAction(
                operateArm(),
                operateDrivetrain(),
                operatePayload(),
                operateTelemetry(),
                handleAutomations(),
                operateGamepads()
        );
    }


    private Action collectSampleAutomation(){
        return new SequentialAction();
    }
    private Action scoreSpecimenAutomation(){
        return new SequentialAction();
    }

    private class HandleAutomations implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                chooseAutomation(AutomationFlag.SCORE_SPECIMEN);
            }
            else if (gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
                chooseAutomation(AutomationFlag.COLLECT_SAMPLE);
            }

            if ((isRobotAutomating() && !currentAutomation.run(telemetryPacket)) || automationStopRequested){
                    chooseAutomation(AutomationFlag.NONE);
                    automationStopRequested = false;
            }

            return true;
        }
    }
    private Action handleAutomations(){
        return new HandleAutomations();
    }

    private class OperateTelemetry implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            telemetry.addData("heading", drivetrain.getHeading());
            telemetry.update();
            return true;
        }
    }
    private Action operateTelemetry(){
        return new OperateTelemetry();
    }

    private class OperateGamepads implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();
            return true;
        }
    }
    private Action operateGamepads(){
        return new OperateGamepads();
    }

    private class OperateArm implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            boolean automationStopRequestedByArm = isSystemAutomating(RobotSystem.ARM);

            double manualPower = gamepadEx2.gamepad.right_trigger - gamepadEx2.gamepad.left_trigger;

            if (Math.abs(manualPower) > TRIGGER_DOWN_TOLERANCE) {
                arm.operateManually(manualPower);
            } else if (gamepadEx2.wasJustPressed(GamepadKeys.Button.X)) {
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
            else automationStopRequestedByArm = false;

            automationStopRequested = automationStopRequested || automationStopRequestedByArm;

            arm.operate();
            return true;
        }
    }
    private Action operateArm(){
        return new OperateArm();
    }

    private class OperateDrivetrain implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
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
            return true;
        }
    }
    private Action operateDrivetrain(){
        return new OperateDrivetrain();
    }

    private class OperatePaylaod implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            boolean automationStopRequestedByPayload = isSystemAutomating(RobotSystem.PAYLOAD);

            // Control payload intake/unload with bumpers on gamepad2
            if (gamepadEx2.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                payload.unload();
            } else if (gamepadEx2.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                payload.intake();
            }
            else automationStopRequestedByPayload = false;

            automationStopRequested = automationStopRequested || automationStopRequestedByPayload;

            payload.operate();
            return true;
        }
    }
    private Action operatePayload(){
        return new OperatePaylaod();
    }


}
