package org.firstinspires.ftc.teamcode.drive.Teleop;


import  com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.motors.RevRoboticsUltraPlanetaryHdHexMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Elevator;
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Servos;
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Slide;

@Config
@TeleOp(name = "V2")
public class V2 extends LinearOpMode {

    Elevator elevator = null;
    Servos servos = null;
    Slide slide = null;

    public static double speed = 0.9;
    public static double turn = 0.6;


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, telemetry);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);


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
            boolean LStick =gamepad1.left_stick_button;
            boolean RStick =gamepad1.right_stick_button;
            double RTG1 = gamepad1.right_trigger;
            double LTG1 = gamepad1.left_trigger;


        while (opModeInInit()) {

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
            Vector2d input = new Vector2d(Math.pow(-gamepad1.left_stick_y,3), Math.pow(-gamepad1.left_stick_x,3)).rotated(-poseEstimate.getHeading());

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

            if(LTG1>0.7 )    // slow mode
            {
                speed = 0.4;
                turn = 0.2;
            } else{
                speed = 0.9;
                turn = 0.6;
            }

            if(LStick || RStick){
                drive.setPoseEstimate(startPose);
            }

            if (BACK1) {                    // home position

                Servos.Arm.goActiveStable();
                Servos.Arm.goInit();
                Servos.Rotate.rotatePick();
                elevator.extendToSlow(elevator.POSITIONS[elevator.HOME]);
                slide.extendToSlow(slide.POSITIONS[slide.HOME]);

            }

            // CONE TRANSFER

            else if (LB1) {      // cone pickup
                Servos.Gripper.Unlock();
                Servos.Arm.goActivePick();
                Servos.Rotate.rotatePick();
                Servos.Gripper.openGripper();
                sleep(300);
                Servos.Arm.goPick();
                sleep(300);
                slide.extendToSlow(slide.POSITIONS[slide.TELETEST]);
            }
            else if (RB1) {           // cone transfer
                slide.extendTo(slide.POSITIONS[slide.TELETEST]+100);
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

            else if (gamepad2.left_bumper) {      // cone pickup
                Servos.Gripper.Unlock();
                Servos.Arm.goActivePick();
                Servos.Rotate.rotatePick();
                Servos.Gripper.openGripper();
                sleep(200);
                Servos.Arm.goPick();
                slide.extendTo(slide.POSITIONS[slide.TELEMAX]);
            }

            else if (gamepad2.right_bumper) {           // cone transfer from far

                slide.extendTo(slide.POSITIONS[slide.TELEMAX]+100);
                sleep(100);
                Servos.Gripper.closeGripper();
                sleep(200);
                Servos.Arm.goInit();
                Servos.Arm.goActiveStable();
                slide.extendTo(slide.POSITIONS[slide.HOME]);
                sleep(300);
                Servos.Rotate.rotateDrop();
                Servos.Arm.goActiveDrop();
                sleep(400);
                Servos.Arm.goDrop();
                sleep(300);
                Servos.Gripper.openGripper();
                sleep(100);
                Servos.Gripper.Lock();
                Servos.Arm.goInit();
                Servos.Rotate.rotatePick();
                sleep(200);
                Servos.Arm.goActiveStable();
            }

            else if (B1) {           // cone transfer

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
                sleep(200);
                Servos.Arm.goActiveStable();
            }

            //************************ UP DOWN BUTTONS *******************************

            else if (A1) {
                elevator.extendTo(elevator.POSITIONS[elevator.LOW_POLE]);
            }
            else if (Y1) {
                elevator.extendTo(elevator.POSITIONS[elevator.MID_POLE]);
            }
            else if (UP1) {
                elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]);
            }
            else if (DOWN1) {
                elevator.extendTo(elevator.POSITIONS[elevator.HOME]);
                Servos.Gripper.Unlock();
            }

            ///////////////////REAL CYCLEE////////////////////////

                else if (START1) {
                slide.extendTo(slide.POSITIONS[slide.TELETEST]+100);
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
                Servos.Arm.goActiveStable();
                sleep(200);
                elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]);
                sleep(700);
                slide.extendToSlow(slide.POSITIONS[slide.TELETEST]);
                Servos.Gripper.Unlock();
                elevator.extendTo(elevator.POSITIONS[elevator.HOME]);
            }

            else if(RIGHT1){
                Servos.Gripper.closeGripper();
                Servos.Arm.goActiveLow();
                Servos.Rotate.rotatePick();
                sleep(300);
                Servos.Arm.goInit();
                slide.extendTo(slide.POSITIONS[slide.HOME]);
            }
            else if(LEFT1){
                Servos.Gripper.openGripper();
                Servos.Arm.goActivePick();
                Servos.Rotate.rotatePick();
                sleep(200);
                Servos.Arm.goPick();
            }
            else if(X1){
                Servos.Arm.goActivePick();
                Servos.Rotate.rotatePick();
                sleep(200);
                Servos.Arm.goPick();
            }
            if(RTG1>0.7)
            {
               Servos.Gripper.openGripper();
            }
            else if(gamepad2.right_trigger>0.7)
            {
                Servos.Gripper.openGripper();
            }
            else if(gamepad2.left_trigger>0.7)
            {
                Servos.Gripper.closeGripper();
            }

            //ManualIncrement
//            if(gamepad2.dpad_down){
//                ElevateRight.setTargetPosition(ElevateRight.getCurrentPosition()-50);
//                ElevateLeft.setTargetPosition(ElevateLeft.getCurrentPosition()-50);
//                ElevateRight.setPower(0.5);
//                ElevateLeft.setPower(0.5);
//            }
//            else if(gamepad2.dpad_up){
//                ElevateRight.setTargetPosition(ElevateRight.getCurrentPosition()+50);
//                ElevateLeft.setTargetPosition(ElevateLeft.getCurrentPosition()+50);
//                ElevateRight.setPower(0.5);
//                ElevateLeft.setPower(0.5);
//            }
//            else if(gamepad2.dpad_left){
//                SlideRight.setTargetPosition(SlideRight.getCurrentPosition()-50);
//                SlideLeft.setTargetPosition(SlideLeft.getCurrentPosition()-50);
//                SlideRight.setPower(0.5);
//                SlideLeft.setPower(0.5);
//            }
//            else if(gamepad2.dpad_right){
//                SlideRight.setTargetPosition(SlideRight.getCurrentPosition()+50);
//                SlideLeft.setTargetPosition(SlideLeft.getCurrentPosition()+50);
//                SlideRight.setPower(0.5);
//                SlideLeft.setPower(0.5);
//            }

            else if(gamepad2.x){
                Servos.Gripper.Lock();
            }
            else if(gamepad2.b){
                Servos.Gripper.Unlock();
            }
        }

    }

}

