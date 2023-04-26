package org.firstinspires.ftc.teamcode.drive.MotionProfile;


import  com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.hardware.motors.RevRoboticsUltraPlanetaryHdHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.drive.MotionProfile.elevator_sub;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Config
@TeleOp(name = "CMDTest")
public class CMDTest extends LinearOpMode {

//    DcMotorEx ElevateLeft = null;
//    DcMotorEx ElevateRight = null;
    elevator_sub elevator_sub = null;
    public static double speed = 0.9;
    public static double turn = 0.6;
    public static double HIGH=1100;

    

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

//        ElevateLeft = hardwareMap.get(DcMotorEx.class, "ElevateLeft");
//        ElevateRight = hardwareMap.get(DcMotorEx.class, "ElevateRight");
//
//        ElevateLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        ElevateRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            Pose2d poseEstimate = drive.getPoseEstimate();
            Vector2d input = new Vector2d(Math.pow(-gamepad1.left_stick_y, 3), Math.pow(-gamepad1.left_stick_x, 3)).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(new Pose2d(input.getX() * speed, input.getY() * speed, -gamepad1.right_stick_x * turn));
            drive.update();

            if (gamepad1.left_trigger > 0.7)    // slow mode
            {
                speed = 0.4;
                turn = 0.2;
            } else {
                speed = 0.9;
                turn = 0.6;
            }

            if (gamepad1.dpad_up)
            {
                elevator_sub.extendTo(elevator_sub.HIGH_POLE);
            } else if (gamepad1.dpad_down) {
                elevator_sub.extendTo(elevator_sub.HOME);
            }

        }
    }
}
