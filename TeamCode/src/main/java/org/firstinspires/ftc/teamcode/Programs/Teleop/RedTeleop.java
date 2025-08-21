package org.firstinspires.ftc.teamcode.Programs.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.GlobalData;

@TeleOp(name="RedTeleop", group="Main Teleop")  // Registers this as a TeleOp mode named "RedTeleop"
public class RedTeleop extends LinearOpMode {

    private TeleopController teleopController;  // Handles the main TeleOp logic

    private double HEADING_OFFSET = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize TeleopController with hardware map, gamepads, and RED alliance
        teleopController = new TeleopController(
                this,
                GlobalData.Alliance.RED,
                HEADING_OFFSET
        );

        // Wait for the driver to press START on the Driver Station
        waitForStart();

        // Main loop runs while OpMode is active (until STOP is pressed)
        while (opModeIsActive()) {
            teleopController.operate();  // Execute TeleOp periodic actions
        }
    }
}
