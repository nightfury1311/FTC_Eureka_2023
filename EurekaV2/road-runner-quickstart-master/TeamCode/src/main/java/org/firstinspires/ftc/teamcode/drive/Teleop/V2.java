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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Servos;
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Slide;

@Config
@TeleOp(name = "V2")
public class V2 extends LinearOpMode {

    Servos servos = null;
    Slide slide = null;

    private PIDController controller;
    public static double p = 0.008, i = 0, d = 0;
    public static double f = 0, ff = 0.14;

    public static int target = 0;
    private DcMotorEx ElevateLeft, ElevateRight;

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
        slide = new Slide(hardwareMap, telemetry);
        controller= new PIDController(p,i,d);

        telemetry= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ElevateLeft = hardwareMap.get(DcMotorEx.class, "ElevateLeft");
        ElevateRight = hardwareMap.get(DcMotorEx.class, "ElevateRight");
        ElevateLeft.setDirection(DcMotorEx.Direction.REVERSE);
        ElevateRight.setDirection(DcMotorEx.Direction.REVERSE);


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
            slide.extendTo(slide.POSITIONS[slide.HOME]);

        }

        while (opModeIsActive()) {
            controller.setPID(p, i, d);

            int ElevateFinalPos = ElevateRight.getCurrentPosition();
            double power = Range.clip(((controller.calculate(ElevateFinalPos, target)+ ff)) , -0.7, 0.8);
            ElevateLeft.setPower(power);
            ElevateRight.setPower(power);

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
                target=0;
                slide.extendToSlow(slide.POSITIONS[slide.HOME]);

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
                slide.extendToSlow(slide.POSITIONS[slide.TELETEST]);
            }

            if (RB1 && RB1Flag == 0) {
                if (slide.getPosition() > 200) {     // Picking with extension
                    RB1Flag = 1;
                    slide.extendTo(slide.POSITIONS[slide.TELETEST] + 100);
                    sleep(100);
                    Servos.Gripper.closeGripper();
                    sleep(200);
                    Servos.Arm.goInit();
                    Servos.Arm.goActiveStable();
                    slide.extendTo(slide.POSITIONS[slide.HOME]);
                    sleep(200);
                    Servos.Rotate.rotateDrop();
                    Servos.Arm.goActiveDrop();
                    sleep(400);
                    Servos.Arm.goDrop();
                    sleep(500);
                    Servos.Gripper.openGripper();
                    sleep(100);
                    Servos.Gripper.Lock();
                    Servos.Arm.goInit();
                    Servos.Rotate.rotatePick();
                    sleep(200);
                    Servos.Arm.goActiveStable();
                }
                else if (slide.getPosition() < 100 && Servos.Arm.armState  == "PICKTELE") {   // Picking without extension
                    RB1Flag = 2;
                    Servos.Gripper.closeGripper();
                    sleep(200);
                    Servos.Rotate.rotateDrop();
                    Servos.Arm.goActiveDrop();
                    Servos.Arm.goDrop();
                    sleep(800);
                    Servos.Gripper.openGripper();
                    sleep(200);
                    Servos.Arm.goInit();
                    Servos.Gripper.Lock();
                    Servos.Rotate.rotatePick();
                    Servos.Arm.goActiveStable();
                }
                else if (Servos.Arm.armState == "ActiveLow" || Servos.Arm.armState == "INIT") {   // Dropping from arm home position
                    RB1Flag = 3;
                    Servos.Rotate.rotateDrop();
                    Servos.Arm.goActiveDrop();
                    sleep(200);
                    Servos.Arm.goDrop();
                    sleep(300);
                    Servos.Gripper.openGripper();
                    sleep(200);
                    Servos.Gripper.Lock();
                    Servos.Arm.goInit();
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

                target=190;
            }

            if (Y1) {

                target=450;
            }

            if (UP1) {
                target=750;
            }

            if (DOWN1) {
                target=0;
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
                slide.extendTo(slide.POSITIONS[slide.HOME]);
            }

            // High junction cycle

            if (START1) {
                slide.extendTo(slide.POSITIONS[slide.TELETEST] + 100);
                sleep(100);
                Servos.Gripper.closeGripper();
                sleep(200);
                Servos.Arm.goInit();
                Servos.Arm.goActiveStable();
                slide.extendTo(slide.POSITIONS[slide.HOME]);
                sleep(200);
                Servos.Rotate.rotateDrop();
                Servos.Arm.goActiveDrop();
                sleep(400);
                Servos.Arm.goDrop();
                sleep(500);
                Servos.Gripper.openGripper();
                sleep(100);
                Servos.Gripper.Lock();
                Servos.Arm.goPickTele();
                Servos.Rotate.rotatePick();
                Servos.Arm.goActiveStable();
                sleep(200);
                target=750;
                sleep(700);
                slide.extendToSlow(slide.POSITIONS[slide.TELETEST]);
                Servos.Gripper.Unlock();
                target=0;
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
            telemetry.addData("FinalPos", ElevateFinalPos);
            telemetry.addData("target", target);
            telemetry.addData("Current ElevateRight", ElevateRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Current ElevateLeft", ElevateLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.update();

        }

    }

}

