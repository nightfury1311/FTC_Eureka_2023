package org.firstinspires.ftc.teamcode.drive.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Elevator;
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Servos;
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Slide;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class LeftSafeHigh extends LinearOpMode
{
    Elevator elevator = null;
    Servos servos = null;
    Slide slide = null;


    OpenCvCamera camera;
    org.firstinspires.ftc.teamcode.Vision.AprilTagDetectionPipeline AprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family

    int PARKING_ZONE1 = 0, PARKING_ZONE2 = 1, PARKING_ZONE3 = 2;

    int[] eureka_IDS = {3,7,9};

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException
    {
        Pose2d PARKING1 = new Pose2d(13, -21, Math.toRadians(90));
        Pose2d PARKING2 = new Pose2d(-36, 27, Math.toRadians(90));
        Pose2d PARKING3 = new Pose2d(-60, 12, Math.toRadians(90));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Camera"), cameraMonitorViewId);
        AprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(AprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        elevator = new Elevator(hardwareMap, telemetry);
        servos = new Servos(hardwareMap, telemetry);
        slide = new Slide(hardwareMap, telemetry);

        elevator.reset();
        slide.reset();
        Servos.Gripper.Lock();
        Servos.Gripper.closeGripper();
        Servos.Arm.goActiveStable();
        Servos.Arm.goDrop();
        Servos.Rotate.rotatePick();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, telemetry);
        Pose2d startPose = new Pose2d(-31, -64, Math.toRadians(90));    //63 y value for tis
        drive.setPoseEstimate(startPose);

        TrajectorySequence pre =drive.trajectorySequenceBuilder(startPose)

                ////// DROP PRELOAD
//                .lineToConstantHeading(new Vector2d(12, -55))
//                .lineToLinearHeading(new Pose2d(12,-20, Math.toRadians(196)))  // dropping position

                .lineToLinearHeading(new Pose2d(-12, -58, Math.toRadians(0)))
                .addTemporalMarker(()->{Servos.Arm.goInit();Servos.Gripper.openGripper();})
                .lineToLinearHeading(new Pose2d(-12,-20, Math.toRadians(-16)))
                .addTemporalMarker(()->elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]))
                .waitSeconds(0.7)
                .addTemporalMarker(()->{Servos.Gripper.Unlock();Servos.Arm.goActivePick1();})
                .addTemporalMarker(()->elevator.extendTo(elevator.POSITIONS[elevator.HOME]))
                .waitSeconds(0.05)
                .addTemporalMarker(()->{Servos.Gripper.Unlock();})


                ////// GO TO PICK CONE 1
                .lineToLinearHeading(new Pose2d(-20.5,-12, Math.toRadians(0)))                 //picking position
                .UNSTABLE_addTemporalMarkerOffset(-0.9,()->{slide.extendTo(slide.POSITIONS[slide.CONE1]);Servos.Arm.goPickCone1();})

                .addTemporalMarker(()->{slide.extendTo(slide.POSITIONS[slide.CONE2]);Servos.Gripper.closeGripper();})
                .waitSeconds(0.2)
                .addTemporalMarker(()->Servos.Arm.goInit())
                .waitSeconds(0.2)
                .addTemporalMarker(()->{slide.extendTo(slide.POSITIONS[slide.HOME]);Servos.Rotate.rotateDrop();Servos.Arm.goActiveDrop();})
                .UNSTABLE_addTemporalMarkerOffset(0.6,()->Servos.Arm.goDrop())
                .UNSTABLE_addTemporalMarkerOffset(0.8,()->Servos.Gripper.openGripper())
                .UNSTABLE_addTemporalMarkerOffset(0.85,()->{Servos.Gripper.Lock();Servos.Arm.goInit();Servos.Rotate.rotatePick();})
                .lineToLinearHeading(new Pose2d(-12,-20, Math.toRadians(-16)))


                ///// DROP CONE 1
                .waitSeconds(0.2)
                .addTemporalMarker(()->elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]))
                .waitSeconds(0.7)
                .addTemporalMarker(()->{Servos.Gripper.Unlock();Servos.Arm.goActivePick2();})
                .addTemporalMarker(()->elevator.extendTo(elevator.POSITIONS[elevator.HOME]))
                .waitSeconds(0.05)
                .addTemporalMarker(()->{Servos.Gripper.Unlock();})

                ////// GO TO PICK CONE 2
                .lineToLinearHeading(new Pose2d(-20,-12, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-0.9,()->{slide.extendTo(slide.POSITIONS[slide.CONE2]);Servos.Arm.goPickCone2();})

                .addTemporalMarker(()->{slide.extendTo(slide.POSITIONS[slide.CONE3]);Servos.Gripper.closeGripper();})
                .waitSeconds(0.2)
                .addTemporalMarker(()->Servos.Arm.goInit())
                .waitSeconds(0.2)
                .addTemporalMarker(()->{slide.extendTo(slide.POSITIONS[slide.HOME]);Servos.Rotate.rotateDrop();Servos.Arm.goActiveDrop();})
                .UNSTABLE_addTemporalMarkerOffset(0.6,()->Servos.Arm.goDrop())
                .UNSTABLE_addTemporalMarkerOffset(0.8,()->Servos.Gripper.openGripper())
                .UNSTABLE_addTemporalMarkerOffset(0.87,()->{Servos.Gripper.Lock();Servos.Arm.goInit();Servos.Rotate.rotatePick();})
                .lineToLinearHeading(new Pose2d(-12,-20, Math.toRadians(-16)))

                ///// DROP CONE 2
                .waitSeconds(0.2)
                .addTemporalMarker(()->elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]))
                .waitSeconds(0.7)
                .addTemporalMarker(()->{Servos.Gripper.Unlock();Servos.Arm.goActivePick3();})
                .addTemporalMarker(()->elevator.extendTo(elevator.POSITIONS[elevator.HOME]))
                .waitSeconds(0.05)
                .addTemporalMarker(()->{Servos.Gripper.Unlock();})

                ////// GO TO PICK CONE 3
                .lineToLinearHeading(new Pose2d(-20,-11.3, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-0.9,()->{slide.extendTo(slide.POSITIONS[slide.CONE3]);Servos.Arm.goPickCone3();})

                .addTemporalMarker(()->{slide.extendTo(slide.POSITIONS[slide.CONE4]);Servos.Gripper.closeGripper();})
                .waitSeconds(0.25)
                .addTemporalMarker(()->Servos.Arm.goInit())
                .waitSeconds(0.2)
                .addTemporalMarker(()->{slide.extendTo(slide.POSITIONS[slide.HOME]);Servos.Rotate.rotateDrop();Servos.Arm.goActiveDrop();})
                .UNSTABLE_addTemporalMarkerOffset(0.6,()->Servos.Arm.goDrop())
                .UNSTABLE_addTemporalMarkerOffset(0.8,()->Servos.Gripper.openGripper())
                .UNSTABLE_addTemporalMarkerOffset(0.87,()->{Servos.Gripper.Lock();Servos.Arm.goInit();Servos.Rotate.rotatePick();})
                .lineToLinearHeading(new Pose2d(-12,-19, Math.toRadians(-16)))

                ///// DROP CONE 3
                .waitSeconds(0.2)
                .addTemporalMarker(()->elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]))
                .waitSeconds(0.7)
                .addTemporalMarker(()->{Servos.Gripper.Unlock();Servos.Arm.goActivePick4();})
                .addTemporalMarker(()->elevator.extendTo(elevator.POSITIONS[elevator.HOME]))
                .waitSeconds(0.05)
                .addTemporalMarker(()->{Servos.Gripper.Unlock();})

                ////// GO TO PICK CONE 4
                .lineToLinearHeading(new Pose2d(-20,-11, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-0.9,()->{slide.extendTo(slide.POSITIONS[slide.CONE4]);Servos.Arm.goPickCone4();})

                .addTemporalMarker(()->{slide.extendTo(slide.POSITIONS[slide.CONE4]);Servos.Gripper.closeGripper();})
                .waitSeconds(0.25)
                .addTemporalMarker(()->Servos.Arm.goInit())
                .waitSeconds(0.2)
                .addTemporalMarker(()->{slide.extendTo(slide.POSITIONS[slide.HOME]);Servos.Rotate.rotateDrop();Servos.Arm.goActiveDrop();})
                .UNSTABLE_addTemporalMarkerOffset(0.6,()->Servos.Arm.goDrop())
                .UNSTABLE_addTemporalMarkerOffset(0.8,()->Servos.Gripper.openGripper())
                .UNSTABLE_addTemporalMarkerOffset(0.9,()->{Servos.Gripper.Lock();Servos.Arm.goInit();Servos.Rotate.rotatePick();})
                .lineToLinearHeading(new Pose2d(-12,-19, Math.toRadians(-16)))

                ///// DROP CONE 4
                .waitSeconds(0.25)
                .addTemporalMarker(()->elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]))
                .waitSeconds(0.7)
                .addTemporalMarker(()->{Servos.Gripper.Unlock();Servos.Arm.goActivePick();})
                .addTemporalMarker(()->elevator.extendTo(elevator.POSITIONS[elevator.HOME]))
                .waitSeconds(0.05)
                .addTemporalMarker(()->{Servos.Gripper.Unlock();})


                ////// GO TO PICK CONE 5
                .lineToLinearHeading(new Pose2d(-20,-11, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-0.9,()->{slide.extendTo(slide.POSITIONS[slide.CONE5]);Servos.Arm.goPick();})
                .waitSeconds(0.25)
                .addTemporalMarker(()->{Servos.Gripper.closeGripper();})
                .waitSeconds(0.2)
                .addTemporalMarker(()->Servos.Arm.goInit())
                .waitSeconds(0.3)
                .addTemporalMarker(()->{slide.extendTo(slide.POSITIONS[slide.HOME]);Servos.Rotate.rotateDrop();Servos.Arm.goActiveDrop();})
                .UNSTABLE_addTemporalMarkerOffset(0.6,()->Servos.Arm.goDrop())
                .UNSTABLE_addTemporalMarkerOffset(0.8,()->Servos.Gripper.openGripper())
                .UNSTABLE_addTemporalMarkerOffset(0.9,()->{Servos.Gripper.Lock();Servos.Arm.goInit();Servos.Rotate.rotatePick();})
                .lineToLinearHeading(new Pose2d(-12,-19, Math.toRadians(-16)))

                ///// DROP CONE 5
                .waitSeconds(0.2)
                .addTemporalMarker(()->elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]))
                .waitSeconds(0.7)
                .addTemporalMarker(()->{Servos.Gripper.Unlock();Servos.Arm.goActiveStable();})
                .addTemporalMarker(()->elevator.extendTo(elevator.POSITIONS[elevator.HOME]))
                .waitSeconds(0.05)
                .addTemporalMarker(()->{Servos.Gripper.Unlock();})
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(-12,-12.001, Math.toRadians(90)))

                .build();


        TrajectorySequence goToP1 = drive.trajectorySequenceBuilder((pre.end()))
                .lineToLinearHeading(new Pose2d(-12,-12, Math.toRadians(90)))
                .waitSeconds(0.001)
                .lineToConstantHeading(new Vector2d(-60,-12))
                .build();

        TrajectorySequence goToP2 = drive.trajectorySequenceBuilder((pre.end()))
                .lineToLinearHeading(new Pose2d(-12,-12, Math.toRadians(90)))
                .waitSeconds(0.001)
                .lineToConstantHeading(new Vector2d(-36,-12))
                .build();

        TrajectorySequence goToP3 = drive.trajectorySequenceBuilder((pre.end()))
                .waitSeconds(0.3)
                .lineToLinearHeading(new Pose2d(-13,-20, Math.toRadians(90)))
                .build();

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = AprilTagDetectionPipeline.getLatestDetections();
            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;
                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == eureka_IDS[PARKING_ZONE1] || tag.id == eureka_IDS[PARKING_ZONE2] || tag.id == eureka_IDS[PARKING_ZONE3])
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");
                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");
                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }
            telemetry.update();
            sleep(20);
        }
        drive.followTrajectorySequence(pre);


        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */
        String ParkingZone = "None";
        if(tagOfInterest.id == eureka_IDS[PARKING_ZONE1]){
            ParkingZone = "1";
            drive.followTrajectorySequence(goToP1);
        }
        else if(tagOfInterest.id == eureka_IDS[PARKING_ZONE2]){
            ParkingZone = "2";
            drive.followTrajectorySequence(goToP2);
        }
        else if(tagOfInterest.id == eureka_IDS[PARKING_ZONE3]){
            ParkingZone = "3";
            drive.followTrajectorySequence(goToP3);
        }

        telemetry.addData("Parking Zone: ", ParkingZone);
        while (opModeIsActive()) {sleep(20);}
        telemetry.addData("Parking Zone: ", ParkingZone);
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }



}