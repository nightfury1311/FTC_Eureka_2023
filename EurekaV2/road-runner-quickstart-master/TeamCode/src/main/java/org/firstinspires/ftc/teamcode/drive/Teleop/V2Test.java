package org.firstinspires.ftc.teamcode.drive.Teleop;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Elevator;
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Servos;
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Slide;

@Config
@TeleOp(name = "V2Test")
public class V2Test extends LinearOpMode {

    Elevator elevator = null;
    Servos servos = null;
    Slide slide = null;

    public static double speed = 0.9;
    public static double turn = 0.6;
    boolean GRIPFlag = false; // gripping
    boolean RB1Flag = false;  // tranfer with & without sliders
    boolean LEFT1Flag = false; //Pick without sliders + low pole
    boolean X1Flag = false; //Ground junction

    boolean RIGHT1Flag = false;

    @Override
    public void runOpMode() throws InterruptedException {

        elevator = new Elevator(hardwareMap, telemetry);
        servos = new Servos(hardwareMap, telemetry);
        slide = new Slide(hardwareMap, telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, telemetry);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);


        while ((opModeInInit())) {

            Servos.Gripper.Unlock();
            Servos.Gripper.openGripper();
            Servos.Arm.goActiveStable();
            Servos.Arm.goInit();
            Servos.Rotate.rotatePick();
            elevator.extendTo(elevator.POSITIONS[elevator.HOME]);
            slide.extendTo(slide.POSITIONS[slide.HOME]);
        }

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            Pose2d poseEstimate = drive.getPoseEstimate();
            Vector2d input = new Vector2d(Math.pow(-gamepad1.left_stick_y, 3), Math.pow(-gamepad1.left_stick_x, 3)).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(new Pose2d(input.getX() * speed, input.getY() * speed, -gamepad1.right_stick_x * turn));
            drive.update();


//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            -gamepad1.left_stick_y * 1,
//                            -gamepad1.left_stick_x * 1,
//                            -gamepad1.right_stick_x * 0.7
////
//                    ));
//            drive.update();

            // GAMEPAD FUNCTIONALITY
            boolean A1 = gamepad1.a;
            boolean B1 = gamepad1.b;
            boolean X1 = gamepad1.x;
            boolean Y1 = gamepad1.y;
            boolean UP1 = gamepad1.dpad_up;
            boolean RIGHT1 = gamepad1.dpad_right;
            boolean DOWN1 = gamepad1.dpad_down;
            boolean LEFT1 = gamepad1.dpad_left;
            boolean RB1 = gamepad1.right_bumper;
            boolean LB1 = gamepad1.left_bumper;
            boolean START1 = gamepad1.start;
            boolean BACK1 = gamepad1.back;
            boolean LStick = gamepad1.left_stick_button;
            boolean RStick = gamepad1.right_stick_button;
            double RTG1 = gamepad1.right_trigger;
            double LTG1 = gamepad1.left_trigger;
            if (!RB1) {
                RB1Flag = false;
            }
            if (!LEFT1) {
                LEFT1Flag = false;
            }
            if (!(RTG1 > 0.7)) {
                GRIPFlag = false;
            }

            if (!RIGHT1) {
                RIGHT1Flag = false;
            }
            if (!X1) {
                X1Flag = false;
            }
            if (BACK1) {                    // home position

                Servos.Arm.goActiveStable();
                Servos.Arm.goInit();
                Servos.Rotate.rotatePick();
                elevator.extendToSlow(elevator.POSITIONS[elevator.HOME]);
                slide.extendToSlow(slide.POSITIONS[slide.HOME]);

            }
//            else if (LTG1 > 0.7)    // slow mode
//            {
//                speed = 0.4;
//                turn = 0.2;
//            } else {
//                speed = 0.9;
//                turn = 0.5;
//            }

            if (LStick || RStick) {
                drive.setPoseEstimate(startPose);
            }

            // CONE TRANSFER

            if (LB1) {      // cone pickup
                Servos.Gripper.Unlock();
                Servos.Arm.goActivePick();
                Servos.Rotate.rotatePick();
                Servos.Gripper.openGripper();
                sleep(300);
                Servos.Arm.goPickTele();
                sleep(300);
                slide.extendToSlow(slide.POSITIONS[slide.TELETEST]);
            }
            if (RB1 && !RB1Flag) {
                RB1Flag = true;
                if (slide.getPosition() > 300) {     // picking with long extension
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
                else if (slide.getPosition() < 100) {   // picking without extension
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
            }
//            else if (RB1) {           // cone transfer
//
//                slide.extendTo(slide.POSITIONS[slide.TELETEST]+100);
//                sleep(100);
//                Servos.Gripper.closeGripper();
//                sleep(200);
//                Servos.Arm.goInit();
//                Servos.Arm.goActiveStable();
//                slide.extendTo(slide.POSITIONS[slide.HOME]);
//                sleep(200);
//                Servos.Rotate.rotateDrop();
//                Servos.Arm.goActiveDrop();
//                sleep(400);
//                Servos.Arm.goDrop();
//                sleep(500);
//                Servos.Gripper.openGripper();
//                sleep(100);
//                Servos.Gripper.Lock();
//                Servos.Arm.goInit();
//                Servos.Rotate.rotatePick();
//                sleep(200);
//                Servos.Arm.goActiveStable();
//            }


            if (B1) {           // cone transfer from low pole position

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

            //************************ UP DOWN BUTTONS *******************************

            if (A1) {
                elevator.extendTo(elevator.POSITIONS[elevator.LOW_POLE]);
            } if (Y1) {
                elevator.extendTo(elevator.POSITIONS[elevator.MID_POLE]);
            } if (UP1) {
                elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]);
            } if (DOWN1) {
                elevator.extendTo(elevator.POSITIONS[elevator.HOME]);
                Servos.Gripper.Unlock();
            }

            ///////////////////REAL CYCLEE////////////////////////

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
                elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]);
                sleep(700);
                slide.extendToSlow(slide.POSITIONS[slide.TELETEST]);
                Servos.Gripper.Unlock();
                elevator.extendTo(elevator.POSITIONS[elevator.HOME]);
            }

            if (LEFT1 && !LEFT1Flag) {
                LEFT1Flag = true;
                if (Servos.Arm.armState == "INIT") {
                    Servos.Gripper.openGripper();
                    Servos.Arm.goActivePick();
                    Servos.Rotate.rotatePick();
                    sleep(200);
                    Servos.Arm.goPickTele();
                } else if (Servos.Arm.armState  == "PICKTELE") {
                    Servos.Gripper.closeGripper();
                    sleep(200);
                    Servos.Arm.goInit();
                    Servos.Arm.goActiveLow();
                    slide.extendTo(slide.POSITIONS[slide.HOME]);
                }
            }

//            else if (RIGHT1) {
//                Servos.Gripper.closeGripper();
//                sleep(200);
//                Servos.Arm.goInit();
//                Servos.Arm.goActiveLow();
//                slide.extendTo(slide.POSITIONS[slide.HOME]);
//            } else if (LEFT1) {
//                Servos.Gripper.openGripper();
//                Servos.Arm.goActivePick();
//                Servos.Rotate.rotatePick();
//                sleep(200);
//                Servos.Arm.goPickTele();
//            }
            if (X1 && !X1Flag) {
                X1Flag = true;
                if (Servos.Gripper.gripperState == "CLOSE") {
                    Servos.Arm.goActivePick();
                    Servos.Rotate.rotatePick();
                    sleep(200);
                    Servos.Arm.goPickTele();
                } else if (Servos.Gripper.gripperState == "OPEN") {
                    Servos.Arm.goActiveStable();
                    Servos.Arm.goInit();
                    Servos.Rotate.rotatePick();
                }
            }
//            else if (X1) {
//                Servos.Arm.goActivePick();
//                Servos.Rotate.rotatePick();
//                sleep(200);
//                Servos.Arm.goPickTele();
//            }
//            if(RTG1>0.7)
//            {
//               Servos.Gripper.openGripper();
//            }

            if (RTG1 > 0.7 && !GRIPFlag) {
                GRIPFlag = true;
                if (Servos.Gripper.gripperState == "CLOSE") {
                    Servos.Gripper.openGripper();
                } else if (Servos.Gripper.gripperState == "OPEN") {
                    Servos.Gripper.closeGripper();
                }
            }
            if (RIGHT1 && !RIGHT1Flag) {
                RIGHT1Flag = true;
                if (Servos.Gripper.gripperState == "UNLOCK") {
                    Servos.Gripper.Lock();
                } else if (Servos.Gripper.gripperState == "LOCK") {
                    Servos.Gripper.Unlock();
                }
            }

        }

    }

}

