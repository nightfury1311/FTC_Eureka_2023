package org.firstinspires.ftc.teamcode.drive.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Elevator;
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Servos;
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Slide;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//@Disabled
@Autonomous
public class RRFH extends LinearOpMode {

    Elevator elevator = null;
    Servos servos = null;
    Slide slide = null;

    @Override

    public void runOpMode() {

        elevator = new Elevator(hardwareMap, telemetry);
        servos = new Servos(hardwareMap, telemetry);
        slide = new Slide(hardwareMap, telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(31, -62, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        elevator.reset();
        slide.reset();
        Servos.Gripper.Lock();
        Servos.Gripper.openGripper();
        Servos.Arm.goActiveStable();
        Servos.Arm.goDrop();
        Servos.Rotate.rotatePick();

        TrajectorySequence pre =drive.trajectorySequenceBuilder(startPose)

//                .lineToConstantHeading(new Vector2d(35, -55))
//                .lineToLinearHeading(new Pose2d(37,-5, Math.toRadians(169)))  // dropping position
                .lineToLinearHeading(new Pose2d(35,-54, Math.toRadians(180)))  // dropping position
                .lineToLinearHeading(new Pose2d(37.5, -3,Math.toRadians(165)))
                .build();
        TrajectorySequence lock1 =drive.trajectorySequenceBuilder(pre.end())
                .lineToLinearHeading(new Pose2d(37.50001, -3.0001, Math.toRadians(165)))
                .build();
        TrajectorySequence lock2 =drive.trajectorySequenceBuilder(lock1.end())
                .lineToLinearHeading(new Pose2d(37.5, -3, Math.toRadians(165)))
                .build();
        TrajectorySequence park =drive.trajectorySequenceBuilder(lock1.end())

                .lineToLinearHeading(new Pose2d(37.5, -14,Math.toRadians(90)))

                .build();


        waitForStart();

        drive.followTrajectorySequence(pre);
        elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]);
        Servos.Arm.goActivePick1();
        Servos.Arm.goPickCone1();
        sleep(400);
        slide.extendTo(slide.POSITIONS[slide.UNSAFE]);
        sleep(400);

        slide.extendTo(slide.POSITIONS[slide.MIN]);
        elevator.extendTo(elevator.POSITIONS[elevator.HOME]);
        sleep(50);
        Servos.Gripper.Unlock();
        sleep(150);

        Servos.Gripper.closeGripper();
        sleep(200);
        Servos.Arm.goInit();
        sleep(300);
        Servos.Rotate.rotateDrop();
        Servos.Arm.goActiveDrop();
        slide.extendTo(slide.POSITIONS[slide.HOME]);
        sleep(400);
        Servos.Arm.goDrop();
        sleep(400);
        Servos.Gripper.openGripper();
        sleep(100);
        Servos.Gripper.Lock();
        Servos.Arm.goPickCone2();
        sleep(100);
        Servos.Rotate.rotatePick();
        Servos.Arm.goActivePick2();

        drive.followTrajectorySequence(lock1);
        elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]);
        sleep(400);
        slide.extendTo(slide.POSITIONS[slide.UNSAFE]);
        sleep(300);
//        Servos.Gripper.Unlock();
//        sleep(100);
        slide.extendTo(slide.POSITIONS[slide.MIN]);
        elevator.extendTo(elevator.POSITIONS[elevator.HOME]);
        sleep(50);
        Servos.Gripper.Unlock();
        sleep(150);

        Servos.Gripper.closeGripper();
        sleep(200);
        Servos.Arm.goInit();
        sleep(300);
        Servos.Rotate.rotateDrop();
        Servos.Arm.goActiveDrop();
        slide.extendTo(slide.POSITIONS[slide.HOME]);
        sleep(400);
        Servos.Arm.goDrop();
        sleep(400);
        Servos.Gripper.openGripper();
        sleep(100);
        Servos.Gripper.Lock();
        Servos.Arm.goPickCone3();
        sleep(100);
        Servos.Rotate.rotatePick();
        Servos.Arm.goActivePick3();

        drive.followTrajectorySequence(lock2);
        elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]);
        sleep(400);
        slide.extendTo(slide.POSITIONS[slide.UNSAFE]);
        sleep(300);
//        Servos.Gripper.Unlock();
//        sleep(100);
        slide.extendTo(slide.POSITIONS[slide.MIN]);
        elevator.extendTo(elevator.POSITIONS[elevator.HOME]);
        sleep(50);
        Servos.Gripper.Unlock();
        sleep(150);

        Servos.Gripper.closeGripper();
        sleep(200);
        Servos.Arm.goInit();
        sleep(300);
        Servos.Rotate.rotateDrop();
        Servos.Arm.goActiveDrop();
        slide.extendTo(slide.POSITIONS[slide.HOME]);
        sleep(400);
        Servos.Arm.goDrop();
        sleep(250);
        Servos.Gripper.openGripper();
        sleep(100);
        Servos.Gripper.Lock();
        Servos.Arm.goPickCone4();
        sleep(100);
        Servos.Rotate.rotatePick();
        Servos.Arm.goActivePick4();
        drive.followTrajectorySequence(lock1);
        elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]);
        sleep(400);
        slide.extendTo(slide.POSITIONS[slide.UNSAFE]);
        sleep(300);
//        Servos.Gripper.Unlock();
//        sleep(100);
        slide.extendTo(slide.POSITIONS[slide.MIN]);
        elevator.extendTo(elevator.POSITIONS[elevator.HOME]);
        sleep(50);
        Servos.Gripper.Unlock();
        sleep(150);

        Servos.Gripper.closeGripper();
        sleep(200);
        Servos.Arm.goInit();
        sleep(300);
        Servos.Rotate.rotateDrop();
        Servos.Arm.goActiveDrop();
        slide.extendTo(slide.POSITIONS[slide.HOME]);
        sleep(400);
        Servos.Arm.goDrop();
        sleep(250);
        Servos.Gripper.openGripper();
        sleep(100);
        Servos.Gripper.Lock();
        Servos.Arm.goPick();
        sleep(100);
        Servos.Rotate.rotatePick();
        Servos.Arm.goActivePick();
        drive.followTrajectorySequence(lock2);
        elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]);
        sleep(400);
        slide.extendTo(slide.POSITIONS[slide.UNSAFE]);
        sleep(300);
//        Servos.Gripper.Unlock();
//        sleep(100);
        slide.extendTo(slide.POSITIONS[slide.MIN]);
        elevator.extendTo(elevator.POSITIONS[elevator.HOME]);
        sleep(50);
        Servos.Gripper.Unlock();
        sleep(150);

        Servos.Gripper.closeGripper();
        sleep(200);
        Servos.Arm.goInit();
        sleep(100);
        Servos.Rotate.rotateDrop();
        Servos.Arm.goActiveDrop();
        slide.extendTo(slide.POSITIONS[slide.HOME]);
        sleep(400);
        Servos.Arm.goDrop();
        sleep(400);
        Servos.Gripper.openGripper();
        sleep(100);
        Servos.Gripper.Lock();
        Servos.Arm.goInit();
        sleep(100);
        Servos.Rotate.rotatePick();
        Servos.Arm.goActiveStable();
        drive.followTrajectorySequence(lock1);
        elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]);
        sleep(700);
//        Servos.Gripper.Unlock();
//        sleep(100);
        elevator.extendTo(elevator.POSITIONS[elevator.HOME]);
        sleep(50);
        Servos.Gripper.Unlock();
        sleep(400);

        drive.followTrajectorySequence(park);



    }

}