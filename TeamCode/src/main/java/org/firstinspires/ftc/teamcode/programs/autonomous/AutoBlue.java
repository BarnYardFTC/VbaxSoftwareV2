package org.firstinspires.ftc.teamcode.programs.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.GlobalData;
import org.firstinspires.ftc.teamcode.components.roadRunner.MecanumDrive;

@Autonomous(name = "nigger", group = "oppressed")
public class AutoBlue extends LinearOpMode {
    private static final Pose2d START_POSE = new Pose2d(0, -60, Math.toRadians(270));
    private static final Pose2d SCORE_POSE = new Pose2d(0, -30, Math.toRadians(270));
    private AutoController controller;


    private Action buildStrafeToLinearHeading(MecanumDrive drive, Pose2d start, Pose2d end){
        return drive.actionBuilder(start).strafeToLinearHeading(end.position, end.heading).build();

    }

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive mecanum = new MecanumDrive(hardwareMap, START_POSE);
        Action traj = buildStrafeToLinearHeading(mecanum, START_POSE, SCORE_POSE);
        controller = new AutoController(this, GlobalData.Alliance.BLUE, START_POSE);
        waitForStart();
        if(isStopRequested())return;
        Actions.runBlocking(new ParallelAction(
                controller.operate(),
                new SequentialAction(new ParallelAction(
                        traj,
                        controller.moveArmTo(Arm.PREPARE_SPECIMEN)),
                        controller.moveArmTo(Arm.SCORE_SPECIMEN)),
                        controller.unloadSample(),
                        controller.moveArmTo(Arm.DEFAULT))
        );
    }


}
