package com.example.meepmeep;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass {

    private static final Pose2d START_POS = new Pose2d(0, -70, Math.toRadians(270));
    private static final Pose2d SCORE_POS = new Pose2d(0, -34, Math.toRadians(270));

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);
        RoadRunnerBotEntity robot = new DefaultBotBuilder(meepMeep).setConstraints(100, 100, Math.toRadians(720), Math.toDegrees(720), 14).build();
        robot.setDimensions(18, 18);

        Action trajectory = buildStrafeToLinearHeading(robot, START_POS, SCORE_POS);

        robot.runAction(new SequentialAction(trajectory));

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(robot)
                .start();
    }

    private static Action buildStrafeToLinearHeading(RoadRunnerBotEntity robot, Pose2d start, Pose2d end){
        return robot.getDrive()
                .actionBuilder(start)
                .strafeToLinearHeading(end.position, end.heading)
                .build();
    }
}