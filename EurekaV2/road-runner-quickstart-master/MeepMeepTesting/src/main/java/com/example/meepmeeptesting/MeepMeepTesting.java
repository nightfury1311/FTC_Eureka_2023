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
                                drive.trajectorySequenceBuilder(new Pose2d(-30, -62, Math.toRadians(90)))
                                        .lineToLinearHeading(new Pose2d(-35,-54, Math.toRadians(0)))  // dropping position
                                        .waitSeconds(0.5)
                                        .lineToLinearHeading(new Pose2d(-35, -19,Math.toRadians(-15)))
                                        .waitSeconds(5)


                                        //PRELOAD
//
//                                        .waitSeconds(1)
//
//
//                                        .lineToLinearHeading(new Pose2d(14,-12, Math.toRadians(180)))
//                                        //1st Cone Pick
//                                        .waitSeconds(2)
//                                        .lineToLinearHeading(new Pose2d(12,-20, Math.toRadians(196)))
//                                        //1st Cone Drop
//                                        .waitSeconds(1)
//
//                                        .lineToLinearHeading(new Pose2d(14,-12, Math.toRadians(180)))
//                                        //2nd Cone Pick
//                                        .waitSeconds(2)
//                                        .lineToLinearHeading(new Pose2d(12,-20, Math.toRadians(196)))
//                                        //2nd Cone Drop
//                                        .waitSeconds(1)
//
//                                        .lineToLinearHeading(new Pose2d(14,-12, Math.toRadians(180)))
//                                        //3rd Cone Pick
//                                        .waitSeconds(2)
//                                        .lineToLinearHeading(new Pose2d(12,-20, Math.toRadians(196)))
//                                        //3rd Cone Drop
//                                        .waitSeconds(1)
//
//                                        .lineToLinearHeading(new Pose2d(14,-12, Math.toRadians(180)))
//                                        //4th Cone Pick
//                                        .waitSeconds(2)
//                                        .lineToLinearHeading(new Pose2d(12,-20, Math.toRadians(196)))
//                                        //4th Cone Drop
//                                        .waitSeconds(1)
//
//                                        .lineToLinearHeading(new Pose2d(14,-12, Math.toRadians(180)))
//                                        //5th Cone Pick
//                                        .waitSeconds(2)
//                                        .lineToLinearHeading(new Pose2d(12,-20, Math.toRadians(196)))
//                                        //5th Cone Drop
//                                        .waitSeconds(1)
//
//                                        .build()

//                        drive.trajectorySequenceBuilder(new Pose2d(32, -62, Math.toRadians(180)))
//                                .lineToConstantHeading(new Vector2d(12, -60))
//                                .lineToLinearHeading(new Pose2d(12,-20, Math.toRadians(196)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}