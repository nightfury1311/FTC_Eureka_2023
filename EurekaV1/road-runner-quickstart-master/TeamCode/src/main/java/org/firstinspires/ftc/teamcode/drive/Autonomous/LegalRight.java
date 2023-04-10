package org.firstinspires.ftc.teamcode.drive.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Subsystems.Extension;
import org.firstinspires.ftc.teamcode.drive.Subsystems.Slide;
import org.firstinspires.ftc.teamcode.drive.Subsystems.Servos;
import org.firstinspires.ftc.teamcode.Vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class LegalRight extends LinearOpMode
{
    Extension extension = null;
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
        Pose2d PARKING1 = new Pose2d(-12, 12, Math.toRadians(90));
        Pose2d PARKING2 = new Pose2d(-36, 27, Math.toRadians(90));
        Pose2d PARKING3 = new Pose2d(-60, 12, Math.toRadians(90));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RightCamera"), cameraMonitorViewId);
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

        extension = new Extension(hardwareMap, telemetry);
        servos = new Servos(hardwareMap, telemetry);
        slide = new Slide(hardwareMap, telemetry);

        extension.reset();
        slide.reset();
        slide.extendTo(slide.POSITIONS[slide.HOME]);

        Servos.Rotate.rotatePick();
        Servos.Gripper.closeGripper();
        Servos.Arm.goLegal();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-37.5, 63, Math.toRadians(0));    //63 y value for tis
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 =drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-35, 63))        //63 y value for tis //61 for blue dais //62 for red dais
                .waitSeconds(0.001)
                .lineToLinearHeading(new Pose2d(-35.7, 4, Math.toRadians(344))) //36.3 last night
                .addTemporalMarker(()->slide.extendTo(slide.POSITIONS[slide.PRE]))
                .addTemporalMarker(()->Servos.Rotate.rotateDrop())
                .addTemporalMarker(()-> Servos.Arm.goDrop())
                .waitSeconds(0.4)
                .addTemporalMarker(()->Servos.Gripper.openGripper())
                .waitSeconds(0.2)
                .addTemporalMarker(()->Servos.Arm.goPickCone1())
                .waitSeconds(0.6)
                .addTemporalMarker(()->Servos.Rotate.rotatePick())

                .build();

        TrajectorySequence lock1 =drive.trajectorySequenceBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(-35.71, 4, Math.toRadians(344)))
                .build();
        TrajectorySequence lock2 =drive.trajectorySequenceBuilder(lock1.end())
                .lineToLinearHeading(new Pose2d(-35.7, 4, Math.toRadians(344)))
                .build();

        TrajectorySequence traj2 =drive.trajectorySequenceBuilder(lock1.end())
                .lineToSplineHeading(new Pose2d(-36, 12, Math.toRadians(270)))
                .build();

        TrajectorySequence goToP1 = drive.trajectorySequenceBuilder((traj2.end()))
                .lineToConstantHeading(new Vector2d(PARKING1.getX(), PARKING1.getY()))
                .build();

        TrajectorySequence goToP2 = drive.trajectorySequenceBuilder((traj2.end()))
                .lineToConstantHeading(new Vector2d(PARKING2.getX(), PARKING2.getY()))
                .build();

        TrajectorySequence goToP3 = drive.trajectorySequenceBuilder((traj2.end()))

                .lineToConstantHeading(new Vector2d(PARKING3.getX(), PARKING3.getY()))
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
        Servos.Arm.goInit();
        drive.followTrajectorySequence(traj1);

        Servos.Arm.goPickCone1();
        extension.extendTo(extension.POSITIONS[extension.HIGH_POLE]);
        sleep(1000); //700
        extension.extendToFast(extension.POSITIONS[extension.FIRST]);
        slide.extendTo(slide.POSITIONS[slide.CONE1]);
        sleep(500);  //500
//        extension.extendToFast(extension.POSITIONS[extension.FIRST]);

        Servos.Gripper.closeGripper();
        sleep(200);
        Servos.Arm.goUP();
        sleep(100);
        slide.extendTo(slide.POSITIONS[slide.ULTRAPROMAX]);  //earlier MAX
        sleep(100);
        Servos.Arm.goInit();

        slide.extendTo(slide.POSITIONS[slide.MIN]);
        sleep(200);
        Servos.Rotate.rotateDrop();
        sleep(500);
        Servos.Arm.goDrop();
        sleep(800);   //earlier 1100

        Servos.Gripper.openGripper();
        sleep(200);
        Servos.Arm.goPickCone2();
        sleep(200);

        drive.followTrajectorySequence(lock1);

        extension.extendTo(extension.POSITIONS[extension.HIGH_POLE]);
        Servos.Rotate.rotatePick();
        sleep(1000);
        extension.extendToFast(extension.POSITIONS[extension.HOME]);
        slide.extendTo(slide.POSITIONS[slide.CONE2]);
        sleep(500);
//        extension.extendToFast(extension.POSITIONS[extension.HOME]);


        Servos.Gripper.closeGripper();
        sleep(200);
        Servos.Arm.goUP();
        sleep(100);
        slide.extendTo(slide.POSITIONS[slide.ULTRAPROMAX]);  //earlier MAX
        sleep(100);
        Servos.Arm.goInit();
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
        sleep(1000);
        extension.extendToFast(extension.POSITIONS[extension.HOME]);
        slide.extendTo(slide.POSITIONS[slide.CONE2]);
        sleep(500);
//        extension.extendToFast(extension.POSITIONS[extension.HOME]);

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
        sleep(1000);
        extension.extendToFast(extension.POSITIONS[extension.HOME]);
        slide.extendTo(slide.POSITIONS[slide.CONE3]);
        sleep(500);
//        extension.extendToFast(extension.POSITIONS[extension.HOME]);

        Servos.Gripper.closeGripper();
        sleep(200);
        Servos.Arm.goInit();
        slide.extendTo(slide.POSITIONS[slide.CONE2]);
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
        sleep(1000);
        extension.extendToFast(extension.POSITIONS[extension.HOME]);
        slide.extendTo(slide.POSITIONS[slide.CONE2]);
        sleep(500);
//        extension.extendToFast(extension.POSITIONS[extension.HOME]);

        Servos.Gripper.closeGripper();
        sleep(200);
        Servos.Arm.goInit();
        sleep(300);
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
        sleep(1100);
        extension.extendToFast(extension.POSITIONS[extension.HOME]);

        sleep(100);
        drive.followTrajectorySequence(traj2);


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