package org.firstinspires.ftc.teamcode.programs.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.components.GlobalData;

@TeleOp(name="BlueTeleop", group="Main Teleop")  // Registers this as a TeleOp mode in the Driver Station
public class BlueTeleop extends LinearOpMode {

    private TeleopController teleopController;  // Main controller for TeleOp logic

    private double HEADING_OFFSET = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the TeleopController with:
        // - hardware map for devices
        // - wrapped gamepad inputs for player 1 and player 2
        // - alliance color set to BLUE
        teleopController = new TeleopController(
                this,
                GlobalData.Alliance.BLUE,
                HEADING_OFFSET
        );

        // Wait for the start button to be pressed
        waitForStart();

        // Main TeleOp loop: run until the stop button is pressed
        while (opModeIsActive()) {
            teleopController.operate();  // Execute all TeleOp operations
        }
    }
}
