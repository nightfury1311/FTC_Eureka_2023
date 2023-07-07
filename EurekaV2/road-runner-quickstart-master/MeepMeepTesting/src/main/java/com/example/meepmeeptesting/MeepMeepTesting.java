package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)

                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(14.3, 17)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 11.8)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-31, -64, Math.toRadians(90)))
                                        .lineToLinearHeading(new Pose2d(-12, -58, Math.toRadians(0)))
                                        .lineToLinearHeading(new Pose2d(-12,-20, Math.toRadians(-16)))
                                        .waitSeconds(1)
                                        .setReversed(true)
                                        .splineTo(new Vector2d(-37, -12), Math.toRadians(-12))
                                        .splineTo(new Vector2d(-60, -12), Math.toRadians(0.00))
//                                        .splineToLinearHeading(new Pose2d(30, -10, Math.toRadians(180)), Math.toRadians(196))
//
//                                        .splineToSplineHeading(new Vector2d(60,-14, Math.toRadians(180)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}