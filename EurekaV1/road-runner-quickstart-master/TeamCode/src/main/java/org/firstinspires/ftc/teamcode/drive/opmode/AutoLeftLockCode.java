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

import org.firstinspires.ftc.teamcode.drive.Subsystems.Extension;
import org.firstinspires.ftc.teamcode.drive.Subsystems.Servos;
import org.firstinspires.ftc.teamcode.drive.Subsystems.Slide;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Disabled
@Autonomous
public class AutoLeftLockCode extends LinearOpMode {


    Extension extension = null;
    Servos servos = null;
    Slide slide = null;

    @Override

    public void runOpMode() {
        extension = new Extension(hardwareMap, telemetry);
        servos = new Servos(hardwareMap, telemetry);
        slide = new Slide(hardwareMap, telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(38, 63, Math.toRadians(180));
        drive.setPoseEstimate(startPose);


        extension.reset();
        slide.reset();
        slide.extendTo(slide.POSITIONS[slide.HOME]);
        Servos.Arm.goInit();
        Servos.Rotate.rotatePick();
        Servos.Gripper.openGripper();

        TrajectorySequence traj1 =drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(35, 63))
                .waitSeconds(0.001)
                .lineToLinearHeading(new Pose2d(35.3, 3, Math.toRadians(196)))
                .build();

        TrajectorySequence lock1 =drive.trajectorySequenceBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(35.31, 3.01, Math.toRadians(196)))
                .build();
        TrajectorySequence lock2 =drive.trajectorySequenceBuilder(lock1.end())
                .lineToLinearHeading(new Pose2d(35.3, 3, Math.toRadians(196)))
                .build();

        TrajectorySequence traj2 =drive.trajectorySequenceBuilder(lock1.end())
                .lineToSplineHeading(new Pose2d(36, 12, Math.toRadians(270)))
                .build();


        waitForStart();

        drive.followTrajectorySequence(traj1);
        extension.extendTo(extension.POSITIONS[extension.HIGH_POLE]);
        slide.extendTo(slide.POSITIONS[slide.CONE1]);
        Servos.Arm.goPickCone1();
        sleep(1200);
        extension.extendTo(extension.POSITIONS[extension.FIRST]);

        Servos.Gripper.closeGripper();
        sleep(200);
        slide.extendTo(slide.POSITIONS[slide.ULTRAMID]);  //earlier MAX
        Servos.Arm.goInit();
        sleep(500);
        slide.extendTo(slide.POSITIONS[slide.MIN]);
        Servos.Rotate.rotateDrop();
        sleep(500);
        Servos.Arm.goDrop();
        sleep(1100);   //earlier 1100

        Servos.Gripper.openGripper();
        sleep(200);
        Servos.Arm.goPickCone2();
        sleep(200);

        drive.followTrajectorySequence(lock1);

        extension.extendTo(extension.POSITIONS[extension.HIGH_POLE]);
        Servos.Rotate.rotatePick();
        sleep(300);
        slide.extendTo(slide.POSITIONS[slide.CONE2]);
        sleep(900);
        extension.extendTo(extension.POSITIONS[extension.HOME]);

        Servos.Gripper.closeGripper();
        sleep(200);
        Servos.Arm.goInit();
        slide.extendTo(slide.POSITIONS[slide.MAX]);
        sleep(500);
        slide.extendTo(slide.POSITIONS[slide.MIN]);
        Servos.Rotate.rotateDrop();
        sleep(500);
        Servos.Arm.goDrop();
        sleep(600);

        Servos.Gripper.openGripper();
        sleep(200);
        Servos.Arm.goPickCone3();
        sleep(200);
//***************************************************

        drive.followTrajectorySequence(lock2);

        extension.extendTo(extension.POSITIONS[extension.HIGH_POLE]);
        Servos.Rotate.rotatePick();
        sleep(300);
        slide.extendTo(slide.POSITIONS[slide.CONE3]);
        sleep(900);
        extension.extendTo(extension.POSITIONS[extension.HOME]);

        Servos.Gripper.closeGripper();
        sleep(200);
        Servos.Arm.goInit();
        slide.extendTo(slide.POSITIONS[slide.MAX]);
        sleep(500);
        slide.extendTo(slide.POSITIONS[slide.MIN]);
        Servos.Rotate.rotateDrop();
        sleep(500);
        Servos.Arm.goDrop();
        sleep(600);

        Servos.Gripper.openGripper();
        sleep(200);
        Servos.Arm.goPickCone4();
        sleep(200);

        drive.followTrajectorySequence(lock1);

        extension.extendTo(extension.POSITIONS[extension.HIGH_POLE]);
        Servos.Rotate.rotatePick();
        sleep(300);
        slide.extendTo(slide.POSITIONS[slide.CONE4]);
        sleep(900);
        extension.extendTo(extension.POSITIONS[extension.HOME]);

        Servos.Gripper.closeGripper();
        sleep(200);
        Servos.Arm.goInit();
        slide.extendTo(slide.POSITIONS[slide.MAX]);
        sleep(500);
        slide.extendTo(slide.POSITIONS[slide.MIN]);
        Servos.Rotate.rotateDrop();
        sleep(500);
        Servos.Arm.goDrop();
        sleep(600);

        Servos.Gripper.openGripper();
        sleep(200);
        Servos.Arm.goPickCone5();
        sleep(200);

        drive.followTrajectorySequence(lock2);

        extension.extendTo(extension.POSITIONS[extension.HIGH_POLE]);
        Servos.Rotate.rotatePick();
        sleep(300);
        slide.extendTo(slide.POSITIONS[slide.CONE5]);
        sleep(900);
        extension.extendTo(extension.POSITIONS[extension.HOME]);

        Servos.Gripper.closeGripper();
        sleep(200);
        Servos.Arm.goInit();
        slide.extendTo(slide.POSITIONS[slide.CONE4]);
        sleep(500);
        slide.extendTo(slide.POSITIONS[slide.MIN]);
        Servos.Rotate.rotateDrop();
        sleep(500);
        Servos.Arm.goDrop();
        sleep(600);

        Servos.Gripper.openGripper();
        sleep(200);
        Servos.Arm.goInit();
        sleep(200);

        drive.followTrajectorySequence(lock1);

        extension.extendTo(extension.POSITIONS[extension.HIGH_POLE]);
        Servos.Rotate.rotatePick();
        slide.extendTo(slide.POSITIONS[slide.HOME]);
        sleep(1200);
        extension.extendTo(extension.POSITIONS[extension.HOME]);
        sleep(1000);
        drive.followTrajectorySequence(traj2);


    }
}