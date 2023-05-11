package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Elevator;
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Servos;
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Slide;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//@Disabled
@Autonomous
public class TestTrajRR extends LinearOpMode {

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
        Servos.Arm.goInit();
        Servos.Rotate.rotatePick();

        TrajectorySequence pre =drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(12, -55))
                .lineToLinearHeading(new Pose2d(12,-20, Math.toRadians(196)))
                .build();
        TrajectorySequence pick1 =drive.trajectorySequenceBuilder(pre.end())
                .lineToLinearHeading(new Pose2d(19,-12, Math.toRadians(180)))
                //1st Cone Pick
                .build();
        TrajectorySequence drop1 =drive.trajectorySequenceBuilder(pick1.end())

                .lineToLinearHeading(new Pose2d(12,-20, Math.toRadians(196)))
//                1st Cone Drop
                .build();
        waitForStart();

        drive.followTrajectorySequence(pre);
        elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]);
        sleep(800);
        Servos.Gripper.Unlock();
        sleep(100);
        Servos.Arm.goActivePick();
        elevator.extendTo(elevator.POSITIONS[elevator.HOME]);
        drive.followTrajectorySequence(pick1);
        Servos.Arm.goPick();
        slide.extendTo(slide.POSITIONS[slide.CONE5]);
        sleep(1000);
        Servos.Gripper.closeGripper();
        sleep(300);
        Servos.Arm.goInit();
        Servos.Arm.goActiveStable();
        slide.extendTo(slide.POSITIONS[slide.HOME]);
        sleep(600);
        Servos.Rotate.rotateDrop();
        Servos.Arm.goActiveDrop();
        sleep(300);
        Servos.Arm.goDrop();
        sleep(500);
        Servos.Gripper.openGripper();
        sleep(200);
        Servos.Gripper.Lock();
        Servos.Arm.goInit();
        drive.followTrajectorySequence(drop1);
        elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]);
        sleep(800);
        Servos.Gripper.Unlock();
        sleep(100);
        Servos.Arm.goActivePick();
        Servos.Rotate.rotatePick();
        elevator.extendTo(elevator.POSITIONS[elevator.HOME]);


        drive.followTrajectorySequence(pick1);
        Servos.Arm.goPick();
        slide.extendTo(slide.POSITIONS[slide.CONE5]);
        sleep(1000);
        Servos.Gripper.closeGripper();
        sleep(300);
        Servos.Arm.goInit();
        Servos.Arm.goActiveStable();
        slide.extendTo(slide.POSITIONS[slide.HOME]);
        sleep(600);
        Servos.Rotate.rotateDrop();
        Servos.Arm.goActiveDrop();
        sleep(300);
        Servos.Arm.goDrop();
        sleep(500);
        Servos.Gripper.openGripper();
        sleep(200);
        Servos.Gripper.Lock();
        Servos.Arm.goInit();
        drive.followTrajectorySequence(drop1);
        elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]);
        sleep(800);
        Servos.Gripper.Unlock();
        sleep(100);
        Servos.Arm.goActivePick();
        Servos.Rotate.rotatePick();
        elevator.extendTo(elevator.POSITIONS[elevator.HOME]);

        drive.followTrajectorySequence(pick1);
        Servos.Arm.goPick();
        slide.extendTo(slide.POSITIONS[slide.CONE5]);
        sleep(1000);
        Servos.Gripper.closeGripper();
        sleep(300);
        Servos.Arm.goInit();
        Servos.Arm.goActiveStable();
        slide.extendTo(slide.POSITIONS[slide.HOME]);
        sleep(600);
        Servos.Rotate.rotateDrop();
        Servos.Arm.goActiveDrop();
        sleep(300);
        Servos.Arm.goDrop();
        sleep(500);
        Servos.Gripper.openGripper();
        sleep(200);
        Servos.Gripper.Lock();
        Servos.Arm.goInit();
        drive.followTrajectorySequence(drop1);
        elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]);
        sleep(800);
        Servos.Gripper.Unlock();
        sleep(100);
        Servos.Arm.goActivePick();
        Servos.Rotate.rotatePick();
        elevator.extendTo(elevator.POSITIONS[elevator.HOME]);

        drive.followTrajectorySequence(pick1);
        Servos.Arm.goPick();
        slide.extendTo(slide.POSITIONS[slide.CONE5]);
        sleep(1000);
        Servos.Gripper.closeGripper();
        sleep(300);
        Servos.Arm.goInit();
        Servos.Arm.goActiveStable();
        slide.extendTo(slide.POSITIONS[slide.HOME]);
        sleep(600);
        Servos.Rotate.rotateDrop();
        Servos.Arm.goActiveDrop();
        sleep(300);
        Servos.Arm.goDrop();
        sleep(500);
        Servos.Gripper.openGripper();
        sleep(200);
        Servos.Gripper.Lock();
        Servos.Arm.goInit();
        drive.followTrajectorySequence(drop1);
        elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]);
        sleep(800);
        Servos.Gripper.Unlock();
        sleep(100);
        Servos.Arm.goActivePick();
        Servos.Rotate.rotatePick();
        elevator.extendTo(elevator.POSITIONS[elevator.HOME]);

        drive.followTrajectorySequence(pick1);
        Servos.Arm.goPick();
        slide.extendTo(slide.POSITIONS[slide.CONE5]);
        sleep(1000);
        Servos.Gripper.closeGripper();
        sleep(300);
        Servos.Arm.goInit();
        Servos.Arm.goActiveStable();
        slide.extendTo(slide.POSITIONS[slide.HOME]);
        sleep(600);
        Servos.Rotate.rotateDrop();
        Servos.Arm.goActiveDrop();
        sleep(300);
        Servos.Arm.goDrop();
        sleep(500);
        Servos.Gripper.openGripper();
        sleep(200);
        Servos.Gripper.Lock();
        Servos.Arm.goInit();
        drive.followTrajectorySequence(drop1);
        elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]);
        sleep(800);
        Servos.Gripper.Unlock();
        sleep(100);
        Servos.Arm.goActivePick();
        Servos.Rotate.rotatePick();
        elevator.extendTo(elevator.POSITIONS[elevator.HOME]);



        sleep(5000);


    }
}