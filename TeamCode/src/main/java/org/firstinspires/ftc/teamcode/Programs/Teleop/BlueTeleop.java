package org.firstinspires.ftc.teamcode.Programs.Teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Components.GlobalData;

@TeleOp(name = "blueTeleop", group = "TeleopMain")
public class BlueTeleop extends LinearOpMode {
    private TeleopController teleopController;
    @Override
    public void runOpMode() throws InterruptedException {
        teleopController = new TeleopController(this, GlobalData.Alliance.BLUE, 0);
        waitForStart();
        while (opModeIsActive()) {
            teleopController.operate();
        }
    }
}
