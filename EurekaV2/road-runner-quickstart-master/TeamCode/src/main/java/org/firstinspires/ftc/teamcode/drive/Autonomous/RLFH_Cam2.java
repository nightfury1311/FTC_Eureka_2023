package org.firstinspires.ftc.teamcode.drive.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
public class RLFH_Cam2 extends LinearOpMode
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
        Pose2d PARKING1 = new Pose2d(-60, -24, Math.toRadians(90));
        Pose2d PARKING2 = new Pose2d(-36, -27, Math.toRadians(90));
        Pose2d PARKING3 = new Pose2d(-13, -36, Math.toRadians(90));

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
        Servos.Gripper.openGripper();
        Servos.Arm.goActiveStable();
        Servos.Arm.goDrop();
        Servos.Rotate.rotatePick();
        slide.extendTo(slide.POSITIONS[slide.HOME]);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, telemetry);
        Pose2d startPose = new Pose2d(-30, -62, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence pre =drive.trajectorySequenceBuilder(startPose)

                .lineToLinearHeading(new Pose2d(-35,-54, Math.toRadians(0)))  // dropping position
                .lineToLinearHeading(new Pose2d(-37, -4,Math.toRadians(15)))
                .build();
        TrajectorySequence lock1 =drive.trajectorySequenceBuilder(pre.end())
                .lineToLinearHeading(new Pose2d(-37.00001, -4.0001, Math.toRadians(15)))
                .build();
        TrajectorySequence lock2 =drive.trajectorySequenceBuilder(lock1.end())
                .lineToLinearHeading(new Pose2d(-37.001, -4, Math.toRadians(15)))
                .build();
        TrajectorySequence park =drive.trajectorySequenceBuilder(lock1.end())

                .lineToLinearHeading(new Pose2d(-36, -12,Math.toRadians(90)))

                .build();

        TrajectorySequence goToP1 = drive.trajectorySequenceBuilder((park.end()))
                .lineToConstantHeading(new Vector2d(-60,-12))
                .waitSeconds(0.001)
                .lineToConstantHeading(new Vector2d(-60,-24))
                .build();

        TrajectorySequence goToP2 = drive.trajectorySequenceBuilder((park.end()))
                .lineToConstantHeading(new Vector2d(-36,-24))
                .build();

        TrajectorySequence goToP3 = drive.trajectorySequenceBuilder((park.end()))
                .lineToConstantHeading(new Vector2d(-12,-12))
                .waitSeconds(0.001)
                .lineToConstantHeading(new Vector2d(-12,-24))
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
        elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]);
        Servos.Arm.goActivePick1();
        Servos.Arm.goPickCone1();
        sleep(400);
        slide.extendTo(slide.POSITIONS[slide.UNSAFE]);
        sleep(400);

        slide.extendTo(slide.POSITIONS[slide.MICRO]);
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
        sleep(400);
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
        sleep(400);
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
        sleep(600);
        Servos.Arm.goDrop();
        sleep(300);
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
        sleep(400);
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
        sleep(400);
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
        sleep(600);
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
        sleep(800);
//        Servos.Gripper.Unlock();
//        sleep(100);
        elevator.extendTo(elevator.POSITIONS[elevator.HOME]);
        sleep(50);
        Servos.Gripper.Unlock();
        sleep(400);

        drive.followTrajectorySequence(park);


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