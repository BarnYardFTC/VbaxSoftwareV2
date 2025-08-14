package org.firstinspires.ftc.teamcode.Programs.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.components.GlobalData;

@TeleOp(name = "RedTeleop", group = "TeleopMain")
public class RedTeleop extends LinearOpMode {
    private TeleopController teleopController;
    @Override
    public void runOpMode() throws InterruptedException {
        teleopController = new TeleopController(hardwareMap, new GamepadEx(gamepad1), new GamepadEx(gamepad2), GlobalData.Alliance.RED, 0);
        waitForStart();
        while (opModeIsActive()) {
            teleopController.operate();
        }
    }
}
