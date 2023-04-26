//package org.firstinspires.ftc.teamcode.drive.Teleop;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.drive.Subsystems.Extension;
//import org.firstinspires.ftc.teamcode.drive.Subsystems.Servos;
//import org.firstinspires.ftc.teamcode.drive.Subsystems.Slide;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//@Disabled
//@Autonomous
//public class Eureka_Auto_Mid extends LinearOpMode {
//
//
//    Extension extension = null;
//    Servos servos = null;
//    Slide slide = null;
//
//    @Override
//
//    public void runOpMode() {
//        extension = new Extension(hardwareMap, telemetry);
//        servos = new Servos(hardwareMap, telemetry);
//        slide = new Slide(hardwareMap, telemetry);
//
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        Pose2d startPose = new Pose2d(-37.5, 63, Math.toRadians(0));
//        drive.setPoseEstimate(startPose);
//
//        extension.reset();
//        slide.reset();
//        slide.extendTo(slide.POSITIONS[slide.HOME]);
//        Servos.Arm.goInit();
//        Servos.Rotate.rotatePick();
//        Servos.Gripper.openGripper();
//
//        TrajectorySequence traj1 =drive.trajectorySequenceBuilder(startPose)
//                .lineToConstantHeading(new Vector2d(-35, 63))
//                .waitSeconds(0.001)
//                .lineToLinearHeading(new Pose2d(-34.6, 21.6, Math.toRadians(0)))
//                .turn(Math.toRadians(13))
//                .build();
//
//        waitForStart();
//
//        drive.followTrajectorySequence(traj1);
//        extension.extendTo(extension.POSITIONS[extension.MID_POLE]);
//        slide.extendTo(slide.POSITIONS[slide.ULTRAMID]);
//        Servos.Arm.goPickCone1Mid();
//        sleep(1200);
//        extension.extendTo(extension.POSITIONS[extension.HOME]);
//
//        Servos.Gripper.closeGripper();
//        sleep(200);
//        slide.extendTo(slide.POSITIONS[slide.ULTRAPROMAX]);
//        Servos.Arm.goInit();
//        sleep(500);
//        slide.extendTo(slide.POSITIONS[slide.MIN]);
//        Servos.Rotate.rotateDrop();
//        sleep(600);
//        Servos.Arm.goDrop();
//        sleep(900);
//
//        Servos.Gripper.openGripper();
//        sleep(200);
//        Servos.Arm.goPickCone2();
//        sleep(200);
//
//
//        extension.extendTo(extension.POSITIONS[extension.MID_POLE]);
//        Servos.Rotate.rotatePick();
//        sleep(300);
//        slide.extendTo(slide.POSITIONS[slide.MAXMID]);
//        sleep(900);
//        extension.extendTo(extension.POSITIONS[extension.HOME]);
//
//        Servos.Gripper.closeGripper();
//        sleep(200);
//        slide.extendTo(slide.POSITIONS[slide.ULTRAMID]);
//        Servos.Arm.goInit();
//        sleep(500);
//        slide.extendTo(slide.POSITIONS[slide.MIN]);
//        Servos.Rotate.rotateDrop();
//        sleep(600);
//        Servos.Arm.goDrop();
//        sleep(900);
//
//        Servos.Gripper.openGripper();
//        sleep(200);
//        Servos.Arm.goPickCone2();
//        sleep(200);
//
//        extension.extendTo(extension.POSITIONS[extension.MID_POLE]);
//        Servos.Rotate.rotatePick();
//        sleep(300);
//        slide.extendTo(slide.POSITIONS[slide.CONE1]);
//        sleep(900);
//        extension.extendTo(extension.POSITIONS[extension.HOME]);
//
//        Servos.Gripper.closeGripper();
//        sleep(200);
//        slide.extendTo(slide.POSITIONS[slide.ULTRAMID]);
//        Servos.Arm.goInit();
//        sleep(500);
//        slide.extendTo(slide.POSITIONS[slide.MIN]);
//        Servos.Rotate.rotateDrop();
//        sleep(600);
//        Servos.Arm.goDrop();
//        sleep(900);
//
//        Servos.Gripper.openGripper();
//        sleep(200);
//        Servos.Arm.goPickCone4();
//        sleep(200);
//        extension.extendTo(extension.POSITIONS[extension.MID_POLE]);
//        Servos.Rotate.rotatePick();
//        sleep(300);
//        slide.extendTo(slide.POSITIONS[slide.CONE1]);
//        sleep(900);
//        extension.extendTo(extension.POSITIONS[extension.HOME]);
//
//        Servos.Gripper.closeGripper();
//        sleep(200);
//        slide.extendTo(slide.POSITIONS[slide.ULTRAMID]);
//        Servos.Arm.goInit();
//        sleep(500);
//        slide.extendTo(slide.POSITIONS[slide.MIN]);
//        Servos.Rotate.rotateDrop();
//        sleep(600);
//        Servos.Arm.goDrop();
//        sleep(900);
//
//        Servos.Gripper.openGripper();
//        sleep(200);
//        Servos.Arm.goPickCone5();
//        sleep(200);
//        extension.extendTo(extension.POSITIONS[extension.MID_POLE]);
//        Servos.Rotate.rotatePick();
//        sleep(300);
//        slide.extendTo(slide.POSITIONS[slide.CONE1]);
//        sleep(900);
//        extension.extendTo(extension.POSITIONS[extension.HOME]);
//
//        Servos.Gripper.closeGripper();
//        sleep(200);
//        slide.extendTo(slide.POSITIONS[slide.ULTRAMID]);
//        Servos.Arm.goInit();
//        sleep(500);
//        slide.extendTo(slide.POSITIONS[slide.MIN]);
//        Servos.Rotate.rotateDrop();
//        sleep(600);
//        Servos.Arm.goDrop();
//        sleep(1200);
//
//        Servos.Gripper.openGripper();
//        sleep(200);
//        Servos.Arm.goInit();
//        extension.extendTo(extension.POSITIONS[extension.MID_POLE]);
//        Servos.Rotate.rotatePick();
//        sleep(300);
//        slide.extendTo(slide.POSITIONS[slide.HOME]);
//        sleep(900);
//        extension.extendTo(extension.POSITIONS[extension.HOME]);
//        sleep(2000);
//
//    }
//}