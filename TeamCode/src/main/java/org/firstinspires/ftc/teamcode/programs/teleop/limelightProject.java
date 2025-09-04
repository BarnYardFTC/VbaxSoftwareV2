package org.firstinspires.ftc.teamcode.programs.teleop;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.Limelight;

@Autonomous
public class limelightProject extends LinearOpMode {

    Limelight limelight;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = new Limelight(hardwareMap.get(Limelight3A.class, "limelight"));

        waitForStart();
        limelight.start();

        while (opModeIsActive()){
            limelight.operate();
            telemetry.addData("Dx", limelight.getDx());
            telemetry.addData("Dy", limelight.getDy());
            telemetry.addData("Da", limelight.getDa());
            telemetry.update();
        }
    }
}
