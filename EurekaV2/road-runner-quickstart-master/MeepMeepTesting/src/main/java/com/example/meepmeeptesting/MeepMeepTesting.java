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
                                drive.trajectorySequenceBuilder(new Pose2d(30, -62, Math.toRadians(90)))
                                        .lineToConstantHeading(new Vector2d(34, -62))
                                        .splineToLinearHeading(new Pose2d(28, -12, Math.toRadians(115)), Math.toRadians(122.01))
                                        .waitSeconds(1)
                                        .lineToLinearHeading(new Pose2d(50, -12, Math.toRadians(180)))
                                        .waitSeconds(1)
                                        .lineToLinearHeading(new Pose2d(28, -12, Math.toRadians(115)))

//                                        .splineToLinearHeading(new Pose2d(58.67, -12.30, Math.toRadians(-8.18)), Math.toRadians(-8.18))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}