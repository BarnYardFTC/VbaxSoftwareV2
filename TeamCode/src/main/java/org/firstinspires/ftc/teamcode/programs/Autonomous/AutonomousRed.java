package org.firstinspires.ftc.teamcode.programs.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.GlobalData;

@Autonomous(name = "Red Autonomous", group = "AutonomousMain")
public class AutonomousRed extends LinearOpMode {

    private static final Pose2d START_POSE = new Pose2d(0, -60, Math.toRadians(270));
    private static final Pose2d SCORE_POSE = new Pose2d(0, -32, Math.toRadians(270));

    private AutonomousController controller;
    private Action traj;

    @Override
    public void runOpMode() {
        controller = new AutonomousController(hardwareMap, START_POSE, GlobalData.Alliance.RED);
        traj = controller.buildStrafeToLinearHeading(START_POSE, SCORE_POSE);

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        new ParallelAction(
                                controller.moveArmTo(Arm.PREPARE_SPECIMEN),
                                traj
                        ),
                        controller.moveArmTo(Arm.SCORE_SPECIMEN),
                        controller.unloadSample(),
                        controller.moveArmTo(Arm.DEFAULT)
                ),
                controller.operate()
        ));
    }
}
