package org.firstinspires.ftc.teamcode.Programs.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.GlobalData;
import org.firstinspires.ftc.teamcode.components.GlobalData.Alliance;
import org.firstinspires.ftc.teamcode.systems.Drivetrain;
import org.firstinspires.ftc.teamcode.systems.Payload;

public class TeleopController {
    private final double TRIGGER_TOLERANCE = 0.05;
    private Drivetrain drivetrain;
    private Arm arm;
    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;
    private Payload payload;
    public TeleopController(HardwareMap hardwareMap, GamepadEx gamepadEx1, GamepadEx gamepadEx2, Alliance alliance, double headingOffset) {
        this.drivetrain = new Drivetrain(
                hardwareMap.get(DcMotor.class, "leftFront"),
                hardwareMap.get(DcMotor.class, "rightFront"),
                hardwareMap.get(DcMotor.class, "leftBack"),
                hardwareMap.get(DcMotor.class, "rightBack"),
                hardwareMap.get(SparkFunOTOS.class, "otos"), headingOffset
        );

        this.payload = new Payload(
                hardwareMap.get(CRServo.class, "leftServo"),
                hardwareMap.get(CRServo.class, "rightServo"),
                hardwareMap.get(NormalizedColorSensor.class, "colorSensor"),
                alliance
        );

        this.gamepadEx1 = gamepadEx1;
        this.gamepadEx2 = gamepadEx2;

        this.arm = new Arm(
                hardwareMap.get(DcMotor.class, "leftArm"),
                hardwareMap.get(DcMotor.class, "rightArm"), GlobalData.OpmodeType.TELEOP
        );
    }
    public void operate(){
        operateDrivetrain();
        operateArm();
        operatePayload();

        gamepadEx1.readButtons();
        gamepadEx2.readButtons();
    }
    private void operateDrivetrain() {
        drivetrain.operate(-gamepadEx1.getLeftX(), gamepadEx1.getLeftY(), gamepadEx1.getRightX(),
                gamepadEx1.wasJustPressed(GamepadKeys.Button.A), gamepadEx1.wasJustPressed(GamepadKeys.Button.X));
    }
    private void operateArm() {
        double power = gamepadEx2.gamepad.right_trigger - gamepadEx2.gamepad.left_trigger;
        if (Math.abs(power)>TRIGGER_TOLERANCE) arm.operateManual(power);
        if (gamepadEx2.wasJustPressed(GamepadKeys.Button.X)) arm.setTargetAngle(Arm.PREP_SPECIMEN);
        if (gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) arm.setTargetAngle(Arm.SCORE_SPECIMEN);
        if (gamepadEx2.wasJustPressed(GamepadKeys.Button.B)) arm.setTargetAngle(Arm.PREP_SUB);
        if (gamepadEx2.wasJustPressed(GamepadKeys.Button.Y)) arm.setTargetAngle(Arm.COLLECT);
        if (gamepadEx2.wasJustPressed(GamepadKeys.Button.A)) arm.setTargetAngle(Arm.DEFAULT);

        arm.operate();
    }
    private void operatePayload() {
        if (gamepadEx2.isDown(GamepadKeys.Button.RIGHT_BUMPER)) payload.intake();
        if (gamepadEx2.isDown(GamepadKeys.Button.LEFT_BUMPER)) payload.unload();
    }
}
