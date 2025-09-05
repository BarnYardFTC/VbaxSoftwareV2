package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Action;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    private static final Pose2d START_POSE = new Pose2d(0, -60, Math.toRadians(270));
    private static final Pose2d SCORE_POSE = new Pose2d(0, -30, Math.toRadians(270));


    public static void main(String[] args) {



        MeepMeep meepMeep = new MeepMeep(750, 60);
        RoadRunnerBotEntity robot = new DefaultBotBuilder(meepMeep).setConstraints(100, 100, Math.toRadians(720), Math.toRadians(720), 14).build();
        robot.setDimensions(18, 18);

        Action traj = buildStrafeToLinearHeading(robot, START_POSE, SCORE_POSE);


        robot.runAction(new SequentialAction(traj));

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK).setDarkMode(true).setBackgroundAlpha(0.95f).addEntity(robot).start();

    }

    private static Action buildStrafeToLinearHeading(RoadRunnerBotEntity robot, Pose2d start, Pose2d end){
        return robot.getDrive().actionBuilder(start).strafeToLinearHeading(end.position, end.heading).build();

    }

    private static Action biuldSplineToLinearHeading(RoadRunnerBotEntity robot, Pose2d start, Pose2d end){
        return robot.getDrive().actionBuilder(start).splineTo(end.position, end.heading).build();

    }

}
