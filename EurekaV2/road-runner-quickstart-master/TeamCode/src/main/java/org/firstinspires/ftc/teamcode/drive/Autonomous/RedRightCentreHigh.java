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
public class RedRightCentreHigh extends LinearOpMode {

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

                ////// DROP PRELOAD
                .lineToConstantHeading(new Vector2d(12, -55))
                .lineToLinearHeading(new Pose2d(12,-20, Math.toRadians(196)))  // dropping position
                .addTemporalMarker(()->elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]))
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Servos.Gripper.Unlock();Servos.Arm.goActivePick1();})
                .waitSeconds(0.05)
                .addTemporalMarker(()->elevator.extendTo(elevator.POSITIONS[elevator.HOME]))


                ////// GO TO PICK CONE 1
                .lineToLinearHeading(new Pose2d(19,-12, Math.toRadians(180)))                 //picking position
                .UNSTABLE_addTemporalMarkerOffset(-0.8,()->{slide.extendTo(slide.POSITIONS[slide.CONE1]);Servos.Arm.goPickCone1();})

                .addTemporalMarker(()->{slide.extendTo(slide.POSITIONS[slide.CONE2]);Servos.Gripper.closeGripper();})
                .waitSeconds(0.2)
                .addTemporalMarker(()->Servos.Arm.goInit())
                .waitSeconds(0.3)
                .addTemporalMarker(()->{slide.extendTo(slide.POSITIONS[slide.HOME]);Servos.Rotate.rotateDrop();Servos.Arm.goActiveDrop();})
                .UNSTABLE_addTemporalMarkerOffset(0.6,()->Servos.Arm.goDrop())
                .UNSTABLE_addTemporalMarkerOffset(0.8,()->Servos.Gripper.openGripper())
                .UNSTABLE_addTemporalMarkerOffset(0.9,()->{Servos.Gripper.Lock();Servos.Arm.goInit();Servos.Rotate.rotatePick();})
                .lineToLinearHeading(new Pose2d(12,-20, Math.toRadians(196)))


                ///// DROP CONE 1
                .waitSeconds(0.2)
                .addTemporalMarker(()->elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]))
                .waitSeconds(0.7)
                .addTemporalMarker(()->{Servos.Gripper.Unlock();Servos.Arm.goActivePick2();})
                .waitSeconds(0.05)
                .addTemporalMarker(()->elevator.extendTo(elevator.POSITIONS[elevator.HOME]))

                ////// GO TO PICK CONE 2
                .lineToLinearHeading(new Pose2d(19,-12, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-0.8,()->{slide.extendTo(slide.POSITIONS[slide.CONE2]);Servos.Arm.goPickCone2();})

                .addTemporalMarker(()->{slide.extendTo(slide.POSITIONS[slide.CONE3]);Servos.Gripper.closeGripper();})
                .waitSeconds(0.2)
                .addTemporalMarker(()->Servos.Arm.goInit())
                .waitSeconds(0.3)
                .addTemporalMarker(()->{slide.extendTo(slide.POSITIONS[slide.HOME]);Servos.Rotate.rotateDrop();Servos.Arm.goActiveDrop();})
                .UNSTABLE_addTemporalMarkerOffset(0.6,()->Servos.Arm.goDrop())
                .UNSTABLE_addTemporalMarkerOffset(0.8,()->Servos.Gripper.openGripper())
                .UNSTABLE_addTemporalMarkerOffset(0.9,()->{Servos.Gripper.Lock();Servos.Arm.goInit();Servos.Rotate.rotatePick();})
                .lineToLinearHeading(new Pose2d(12,-20, Math.toRadians(196)))

                ///// DROP CONE 2
                .waitSeconds(0.2)
                .addTemporalMarker(()->elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]))
                .waitSeconds(0.7)
                .addTemporalMarker(()->{Servos.Gripper.Unlock();Servos.Arm.goActivePick3();})
                .waitSeconds(0.05)
                .addTemporalMarker(()->elevator.extendTo(elevator.POSITIONS[elevator.HOME]))

                ////// GO TO PICK CONE 3
                .lineToLinearHeading(new Pose2d(19,-12, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-0.8,()->{slide.extendTo(slide.POSITIONS[slide.CONE3]);Servos.Arm.goPickCone3();})

                .addTemporalMarker(()->{slide.extendTo(slide.POSITIONS[slide.CONE4]);Servos.Gripper.closeGripper();})
                .waitSeconds(0.2)
                .addTemporalMarker(()->Servos.Arm.goInit())
                .waitSeconds(0.3)
                .addTemporalMarker(()->{slide.extendTo(slide.POSITIONS[slide.HOME]);Servos.Rotate.rotateDrop();Servos.Arm.goActiveDrop();})
                .UNSTABLE_addTemporalMarkerOffset(0.6,()->Servos.Arm.goDrop())
                .UNSTABLE_addTemporalMarkerOffset(0.8,()->Servos.Gripper.openGripper())
                .UNSTABLE_addTemporalMarkerOffset(0.9,()->{Servos.Gripper.Lock();Servos.Arm.goInit();Servos.Rotate.rotatePick();})
                .lineToLinearHeading(new Pose2d(12,-20, Math.toRadians(196)))

                ///// DROP CONE 3
                .waitSeconds(0.2)
                .addTemporalMarker(()->elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]))
                .waitSeconds(0.7)
                .addTemporalMarker(()->{Servos.Gripper.Unlock();Servos.Arm.goActivePick4();})
                .waitSeconds(0.05)
                .addTemporalMarker(()->elevator.extendTo(elevator.POSITIONS[elevator.HOME]))

                ////// GO TO PICK CONE 4
                .lineToLinearHeading(new Pose2d(19,-12, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-0.8,()->{slide.extendTo(slide.POSITIONS[slide.CONE4]);Servos.Arm.goPickCone4();})

                .addTemporalMarker(()->{slide.extendTo(slide.POSITIONS[slide.CONE4]);Servos.Gripper.closeGripper();})
                .waitSeconds(0.2)
                .addTemporalMarker(()->Servos.Arm.goInit())
                .waitSeconds(0.3)
                .addTemporalMarker(()->{slide.extendTo(slide.POSITIONS[slide.HOME]);Servos.Rotate.rotateDrop();Servos.Arm.goActiveDrop();})
                .UNSTABLE_addTemporalMarkerOffset(0.6,()->Servos.Arm.goDrop())
                .UNSTABLE_addTemporalMarkerOffset(0.8,()->Servos.Gripper.openGripper())
                .UNSTABLE_addTemporalMarkerOffset(0.9,()->{Servos.Gripper.Lock();Servos.Arm.goInit();Servos.Rotate.rotatePick();})
                .lineToLinearHeading(new Pose2d(12,-20, Math.toRadians(196)))

                ///// DROP CONE 4
                .waitSeconds(0.2)
                .addTemporalMarker(()->elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]))
                .waitSeconds(0.7)
                .addTemporalMarker(()->{Servos.Gripper.Unlock();Servos.Arm.goActivePick();})
                .waitSeconds(0.05)
                .addTemporalMarker(()->elevator.extendTo(elevator.POSITIONS[elevator.HOME]))


                ////// GO TO PICK CONE 5
                .lineToLinearHeading(new Pose2d(19,-12, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-0.8,()->{slide.extendTo(slide.POSITIONS[slide.CONE5]);Servos.Arm.goPick();})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Servos.Gripper.closeGripper();})
                .waitSeconds(0.3)
                .addTemporalMarker(()->Servos.Arm.goInit())
                .waitSeconds(0.3)
                .addTemporalMarker(()->{slide.extendTo(slide.POSITIONS[slide.HOME]);Servos.Rotate.rotateDrop();Servos.Arm.goActiveDrop();})
                .UNSTABLE_addTemporalMarkerOffset(0.6,()->Servos.Arm.goDrop())
                .UNSTABLE_addTemporalMarkerOffset(0.8,()->Servos.Gripper.openGripper())
                .UNSTABLE_addTemporalMarkerOffset(0.9,()->{Servos.Gripper.Lock();Servos.Arm.goInit();Servos.Rotate.rotatePick();})
                .lineToLinearHeading(new Pose2d(12,-20, Math.toRadians(196)))

                ///// DROP CONE 4
                .waitSeconds(0.2)
                .addTemporalMarker(()->elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]))
                .waitSeconds(0.7)
                .addTemporalMarker(()->{Servos.Gripper.Unlock();Servos.Arm.goActiveStable();})
                .waitSeconds(0.05)
                .addTemporalMarker(()->elevator.extendTo(elevator.POSITIONS[elevator.HOME]))


                .waitSeconds(5)
                .build();
        waitForStart();

        drive.followTrajectorySequence(pre);


    }

}