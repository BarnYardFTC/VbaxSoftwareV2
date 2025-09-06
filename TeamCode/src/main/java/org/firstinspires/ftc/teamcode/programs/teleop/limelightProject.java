package org.firstinspires.ftc.teamcode.programs.teleop;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.components.Limelight;
import org.firstinspires.ftc.teamcode.systems.Drivetrain;

//i tried doing some shit and its not at all finished. shit is the test shit to make the robot
//go for apriltag in Y axis (and to make real code based on that later -- Maxim
@Autonomous
public class limelightProject extends LinearOpMode {

    Drivetrain drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain = new Drivetrain(
                hardwareMap.get(Limelight3A.class, "limelight"),
                hardwareMap.get(DcMotorEx.class, "leftFront"),
                hardwareMap.get(DcMotorEx.class, "leftBack"),
                hardwareMap.get(DcMotorEx.class, "rightFront"),
                hardwareMap.get(DcMotorEx.class, "rightBack")
        );

        waitForStart();

        drivetrain.start();

        while (opModeIsActive()){
            drivetrain.operateAuto();
            drivetrain.displayData(telemetry);
            telemetry.update();
        }
    }
}

