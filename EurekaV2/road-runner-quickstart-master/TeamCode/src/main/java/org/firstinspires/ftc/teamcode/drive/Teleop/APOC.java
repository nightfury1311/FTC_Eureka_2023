package org.firstinspires.ftc.teamcode.drive.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Servos;
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Slide;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@TeleOp(name = "APOC")
public class APOC extends LinearOpMode {

    Servos servos = null;

    private PIDController controller;
    private PIDController slidercontroller;
    public static double p = 0.008, i = 0, d = 0, ff = 0.14;
    public static int target = 0;
    public static int HIGH = 750;   // HIGH POLE
    public static int MID = 450;   // MID POLE
    public static int LOW = 190;   // LOW POLE
    public static int slidertarget = 0;
    public static int TEST = 275;   // slider extension
    public static int HOME = 0;     // slider + elevator home

    private DcMotorEx ElevateLeft, ElevateRight;
    private DcMotorEx SlideLeft, SlideRight;

    public static double speed = 0.9;
    public static double turn = 0.6;

    // Flags for toggle commands

    int RB1Flag = 0;

    boolean RTG1Flag = false; // Gripping

    boolean X1Flag = false; // Ground junction placement + retraction

    boolean B1Flag = false; // Locking

    @Override
    public void runOpMode() throws InterruptedException {

        servos = new Servos(hardwareMap, telemetry);
        slidercontroller = new PIDController(p,i,d);
        controller= new PIDController(p,i,d);

        telemetry= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ElevateLeft = hardwareMap.get(DcMotorEx.class, "ElevateLeft");
        ElevateRight = hardwareMap.get(DcMotorEx.class, "ElevateRight");
        SlideLeft = hardwareMap.get(DcMotorEx.class, "SlideLeft");
        SlideRight = hardwareMap.get(DcMotorEx.class, "SlideRight");
        ElevateLeft.setDirection(DcMotorEx.Direction.REVERSE);
        ElevateRight.setDirection(DcMotorEx.Direction.REVERSE);
        SlideRight.setDirection(DcMotorSimple.Direction.REVERSE);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, telemetry);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        telemetry= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        while ((opModeInInit())) {
            // Initialization - resetting all motors and servos
            Servos.Gripper.Unlock();
            Servos.Gripper.openGripper();
            Servos.Arm.goActiveStable();
            Servos.Arm.goInit();
            Servos.Rotate.rotatePick();
        }

        TrajectorySequence transfer = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(()->{slidertarget=TEST+100;})
                .waitSeconds(0.05)
                .addTemporalMarker(()->{Servos.Gripper.closeGripper();})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{Servos.Arm.goInit();Servos.Arm.goActiveStable();slidertarget=HOME;})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Servos.Arm.goActiveDrop();Servos.Rotate.rotateDrop();})
                .waitSeconds(0.4)
                .addTemporalMarker(()->{Servos.Arm.goDrop();})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Servos.Gripper.openGripper();})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{Servos.Gripper.Lock();Servos.Arm.goInit();})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{Servos.Rotate.rotatePick();Servos.Arm.goActiveStable();})
                .build();


        TrajectorySequence cycle = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(()->{slidertarget=TEST+100;})
                .waitSeconds(0.05)
                .addTemporalMarker(()->{Servos.Gripper.closeGripper();})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{Servos.Arm.goInit();Servos.Arm.goActiveStable();slidertarget=HOME;})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Servos.Arm.goActiveDrop();Servos.Rotate.rotateDrop();})
                .waitSeconds(0.4)
                .addTemporalMarker(()->{Servos.Arm.goDrop();})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Servos.Gripper.openGripper();})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{Servos.Gripper.Lock();Servos.Arm.goPickTele();})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{Servos.Rotate.rotatePick();Servos.Arm.goActivePick();})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{target=HIGH;})
                .waitSeconds(0.7)
                .addTemporalMarker(()->{slidertarget=TEST-200;})
                .addTemporalMarker(()->{target=HOME;Servos.Gripper.Unlock();})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{slidertarget=TEST;})
                .build();


        while (opModeIsActive()) {
            controller.setPID(p, i, d);
            slidercontroller.setPID(p, i, d);

            int ElevateFinalPos = ElevateRight.getCurrentPosition();
            int SlideFinalPos = SlideRight.getCurrentPosition();
            double power = Range.clip(((controller.calculate(ElevateFinalPos, target)+ ff)) , -0.7, 0.8);
            double sliderpower =  Range.clip(((slidercontroller.calculate(SlideFinalPos, slidertarget)+ ff)) , -0.9, 0.9);
            ElevateLeft.setPower(power);
            ElevateRight.setPower(power);
            SlideLeft.setPower(sliderpower);
            SlideRight.setPower(sliderpower);

            telemetry.addData("Status", "Running");

            //Field centric drive
            Pose2d poseEstimate = drive.getPoseEstimate();
            Vector2d input = new Vector2d(Math.pow(-gamepad1.left_stick_y, 3), Math.pow(-gamepad1.left_stick_x, 3)).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(new Pose2d(input.getX() * speed, input.getY() * speed, -gamepad1.right_stick_x * turn));
            drive.update();

            // GAMEPAD FUNCTIONALITY
            boolean A1 = gamepad1.a;                        // Elevator - Low junction
            boolean B1 = gamepad1.b;                        // Toggling lock state - NOT WORKING
            boolean X1 = gamepad1.x;                        // Ground junctions - NOT WORKING
            boolean Y1 = gamepad1.y;                        // Elevator - Mid junction
            boolean UP1 = gamepad1.dpad_up;                 // Elevator - High junction
            boolean RIGHT1 = gamepad1.dpad_right;           // Cone pickup - Low junction
            boolean DOWN1 = gamepad1.dpad_down;             // Elevator -  Home position
            boolean LEFT1 = gamepad1.dpad_left;             // Cone pickup - Without extension
            boolean RB1 = gamepad1.right_bumper;            // Cone transfer - NOT FULLY WORKING
            boolean LB1 = gamepad1.left_bumper;             // Cone pickup - With extension
            boolean START1 = gamepad1.start;                // High junction cycle
            boolean BACK1 = gamepad1.back;                  // Resetting all motors and servos / Home position
            boolean LStick = gamepad1.left_stick_button;    // Resetting heading
            boolean RStick = gamepad1.right_stick_button;   // Resetting heading
            double RTG1 = gamepad1.right_trigger;           // Toggling gripper state - NOT WORKING
            double LTG1 = gamepad1.left_trigger;            // Slowmode

            // Flags for toggle commands


            // Defining commands/keybinds

            if (LStick || RStick) {         // Resetting heading (for field centric)
                drive.setPoseEstimate(startPose);
            }

            if (BACK1) {                    // Home position

                Servos.Arm.goActiveStable();
                Servos.Arm.goInit();
                Servos.Rotate.rotatePick();
                target=HOME;
                slidertarget=HOME;

            }

            if (LTG1 > 0.7)    // Slow mode
            {
                speed = 0.4;
                turn = 0.2;
            } else {
                speed = 0.9;
                turn = 0.5;
            }

            if (RTG1 > 0.7 && !RTG1Flag) {       // Toggling gripper state
                RTG1Flag = true;
                if (Servos.Gripper.gripperState == "CLOSE") {   // If closed, then open
                    Servos.Gripper.openGripper();
                } else if (Servos.Gripper.gripperState == "OPEN") {     // If open, then close
                    Servos.Gripper.closeGripper();
                }
            }

            if (LB1) {      // Extending sliders for pickup
                Servos.Gripper.Unlock();
                Servos.Arm.goActivePick();
                Servos.Rotate.rotatePick();
                Servos.Gripper.openGripper();
                sleep(300);
                Servos.Arm.goPickTele();
                sleep(300);
                slidertarget=TEST;
            }

            if (RB1 && RB1Flag == 0) {
                if (SlideFinalPos > 200) {     // Picking with extension
                    RB1Flag = 1;
                    drive.followTrajectorySequenceAsync(transfer);
                }
                else if (SlideFinalPos < 100 && Servos.Arm.armState  == "PICKTELE") {   // Picking without extension
                    RB1Flag = 2;
                    Servos.Gripper.closeGripper();
                    sleep(200);
                    Servos.Rotate.rotateDrop();
                    Servos.Arm.goActiveDrop();
                    Servos.Arm.goDrop();
                    sleep(800);
                    Servos.Gripper.openGripper();
                    sleep(100);
                    Servos.Arm.goInit();
                    Servos.Gripper.Lock();
                    sleep(100);
                    Servos.Rotate.rotatePick();
                    Servos.Arm.goActiveStable();
                }
                else if (Servos.Arm.armState == "ActiveLow" || Servos.Arm.armState == "ActiveStable") {   // Dropping from arm home position
                    RB1Flag = 3;
                    Servos.Rotate.rotateDrop();
                    Servos.Arm.goActiveDrop();
                    sleep(200);
                    Servos.Arm.goDrop();
                    sleep(300);
                    Servos.Gripper.openGripper();
                    sleep(100);
                    Servos.Gripper.Lock();
                    Servos.Arm.goInit();
                    sleep(100);
                    Servos.Rotate.rotatePick();
                    Servos.Arm.goActiveStable();
                }
            }

            if (B1 && !B1Flag) {
                B1Flag = true;
                if (Servos.Gripper.lockState == "UNLOCK") {
                    Servos.Gripper.Lock();
                } else if (Servos.Gripper.lockState == "LOCK") {
                    Servos.Gripper.Unlock();
                }
            }

            if (X1 && !X1Flag) {
                X1Flag = true;
                if (Servos.Arm.armState == "ActiveLow" || Servos.Arm.armState == "INIT") {
                    Servos.Arm.goActivePick();
                    Servos.Rotate.rotatePick();
                    sleep(200);
                    Servos.Arm.goPickTele();
                } else if (Servos.Arm.armState == "PICKTELE") {
                    Servos.Arm.goActiveStable();
                    Servos.Arm.goInit();
                    Servos.Rotate.rotatePick();
                }
            }

            // Elevator extensions

            if (A1) {

                target=LOW;
            }

            if (Y1) {

                target=MID;
            }

            if (UP1) {
                target=HIGH;
            }

            if (DOWN1) {
                target=HOME;
                Servos.Gripper.Unlock();
            }

            if (LEFT1) {
                Servos.Gripper.openGripper();
                Servos.Arm.goActivePick();
                Servos.Rotate.rotatePick();
                sleep(200);
                Servos.Arm.goPickTele();
            }

            if (RIGHT1) {
                Servos.Gripper.closeGripper();
                sleep(200);
                Servos.Arm.goInit();
                Servos.Arm.goActiveLow();
                slidertarget=HOME;
            }

            // High junction cycle

            if (START1) {
                drive.followTrajectorySequenceAsync(cycle);
            }
            // Flags
            if (!B1) {
                B1Flag = false;
            }

            if (!X1) {
                X1Flag = false;
            }

            if (RTG1 <= 0.7) {
                RTG1Flag = false;
            }

            if (!RB1) {
                RB1Flag = 0;
            }
            telemetry.addData("ElevateFinalPos", ElevateFinalPos);
            telemetry.addData("Elevatetarget", target);
            telemetry.addData("SliderFinalPos", SlideFinalPos);
            telemetry.addData("Slidetarget", slidertarget);
            telemetry.addData("Current ElevateRight", ElevateRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Current ElevateLeft", ElevateLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Current SlideRight", SlideRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Current SlideLeft", SlideLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.update();

        }

    }

}

