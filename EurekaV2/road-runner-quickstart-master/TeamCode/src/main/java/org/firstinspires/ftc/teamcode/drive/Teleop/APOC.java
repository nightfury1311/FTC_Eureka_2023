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
@Disabled
@Config
@TeleOp(name = "APOC")
public class APOC extends LinearOpMode {

    Elevator elevator = null;
    Servos servos = null;
    Slide slide = null;

    public static double speed = 0.9;
    public static double turn = 0.6;


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


//        ElevateLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        ElevateRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        SlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        SlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator = new Elevator(hardwareMap, telemetry);
        servos = new Servos(hardwareMap, telemetry);
        slide = new Slide(hardwareMap, telemetry);


        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeInInit()) {
            elevator.extendTo(elevator.POSITIONS[elevator.HOME]);
            slide.extendTo(slide.POSITIONS[slide.HOME]);
            Servos.Arm.goInit();
        }
//        waitForStart();
        while (opModeIsActive()) {
        }

            telemetry.addData("Status", "Running");
            Pose2d poseEstimate = drive.getPoseEstimate();
            Vector2d input = new Vector2d(Math.pow(-gamepad1.left_stick_y,3), Math.pow(-gamepad1.left_stick_x,3)).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(new Pose2d(input.getX() * speed, input.getY() * speed, -gamepad1.right_stick_x * turn));
            drive.update();


//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            -gamepad1.left_stick_y * 0.9,
//                            -gamepad1.left_stick_x * 0.9,
//                            -gamepad1.right_stick_x * 0.7
////
//                    ));
//            drive.update();

            if(gamepad1.left_trigger>0.7 )    // slow mode
            {
                speed = 0.4;
                turn = 0.2;
            } else{
                speed = 0.9;
                turn = 0.6;
            }

            //**//**//**//**//**//**//**//**//**//**//**//**//**//**//**//**//**//**//**//**//
            //                                 MAIN CODE                                    //
            //**//**//**//**//**//**//**//**//**//**//**//**//**//**//**//**//**//**//**//**//


            if(gamepad1.dpad_up)
            {
                elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]);
            }
            else if (gamepad1.dpad_down)
            {
                elevator.extendTo(elevator.POSITIONS[elevator.HOME]);
            }


            telemetry.addData("Servo Gripper Position", servos.servoGripper.getPosition());
            telemetry.addData("Servo Left Position", servos.servoLF.getPosition());
            telemetry.addData("Servo Right Position", servos.servoRF.getPosition());
            telemetry.addData("Servo Rotation Position", servos.servoRotate.getPosition());
            telemetry.addData("Current ElevateLeft", elevator.ElevateLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("ElevateLeft", elevator.ElevateLeft.getCurrentPosition());
            telemetry.addData("Current ElevateRight", elevator.ElevateRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("ElevateRight", elevator.ElevateRight.getCurrentPosition());
            telemetry.addData("Current SlideLeft", slide.SlideLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("SlideLeft", slide.SlideLeft.getCurrentPosition());
            telemetry.addData("Current SlideRight", slide.SlideRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("SlideRight", slide.SlideRight.getCurrentPosition());
            telemetry.update();

        }
    }




