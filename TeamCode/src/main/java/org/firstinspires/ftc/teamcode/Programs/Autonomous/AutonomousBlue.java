package org.firstinspires.ftc.teamcode.Programs.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Arm;
import org.firstinspires.ftc.teamcode.Components.GlobalData;
import org.firstinspires.ftc.teamcode.Programs.Teleop.TeleopController;

@Autonomous(name="AutonomousBlue", group="!auto")
public class AutonomousBlue extends LinearOpMode {
    private final Pose2d startPos = new Pose2d(0, -60, Math.toRadians(270));
    private final Pose2d endPos = new Pose2d(0, -32, Math.toRadians(270));
    private Action traj;
    private AutonomousController autonomousController;
    @Override
    public void runOpMode() throws InterruptedException {
        autonomousController = new AutonomousController(this, GlobalData.Alliance.BLUE, startPos);
        traj = autonomousController.buildStrafeToLinearHeading(startPos, endPos);
        waitForStart();
        if (isStopRequested()) return;
        Actions.runBlocking(new ParallelAction(autonomousController.operate(),
                new SequentialAction(
                        new ParallelAction(autonomousController.moveArmTo(Arm.PREP_SPECIMEN),
                                traj),
                        autonomousController.moveArmTo(Arm.SCORE_SPECIMEN),
                        autonomousController.unloadSample(),
                        autonomousController.moveArmTo(Arm.DEFAULT)
                )));
    }

}
