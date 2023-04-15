package org.firstinspires.ftc.teamcode.drive.opmode;


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
@Config
@TeleOp(name = "EurekaCycle")
public class eureka_cycle extends LinearOpMode {
    Servo servoGripper;
    Servo servoLeft;
    Servo servoRight;
    Servo servoRotation;
    DcMotorEx ElevateLeft = null;
    DcMotorEx ElevateRight = null;
    DcMotorEx SlideLeft = null;
    DcMotorEx SlideRight = null;

    //OLD VALUES AT 9:1 elevator ratio
    // public static int LOW = 600;
    //    public static int MID = 1150;
    //    public static int HIGH = 1760;

    // NEW VALUES AT 5:1 elevator ratio
    public static int LOW = 330;   // 320
    public static int MID = 700;   //740           // counts value
    public static int HIGH = 1110; //1160

//    //NEW VALUES AT 3:1 elevator ratio
//    public static int LOW = 183;   // 320
//    public static int MID = 387;   //740           //counts value
//    public static int HIGH = 610; //1160

    public static int MAX = 1150;   // MAX IS 1175 , dont go above this value
    public static int MIN = 290;
    public static int TEST = 1100; //1100   //700 previous value

    public static double GRIPPER_OPEN = 0.32 ;
    public static double GRIPPER_CLOSE = 0;
    public static double PICK_ARM_LEFT = 0.95;
    public static double PICK_ARM_RIGHT = 0.05;
    public static double DROP_ARM_LEFT = 0.1;
    public static double DROP_ARM_RIGHT = 0.9;
    public static double GROUND_ARM_LEFT = 0.8;
    public static double GROUND_ARM_RIGHT = 0.2;
    public static double HOME_ARM = 0.5;
    public static double ROTATE_PICK = 0;
    public static double ROTATE_DROP = 0.65;

    public static double speed = 0.9;
    public static double turn = 0.6;


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        servoGripper = hardwareMap.get(Servo.class, "servoGripper");
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");
        servoRotation = hardwareMap.get(Servo.class, "servoRotation");
        ElevateLeft = hardwareMap.get(DcMotorEx.class, "ElevateLeft");
        ElevateRight = hardwareMap.get(DcMotorEx.class, "ElevateRight");
        SlideLeft = hardwareMap.get(DcMotorEx.class, "SlideLeft");
        SlideRight = hardwareMap.get(DcMotorEx.class, "SlideRight");


        ElevateLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElevateRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SlideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        SlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);

//        ElevateLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        ElevateRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        SlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        SlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeInInit()) {
            servoGripper.setPosition(GRIPPER_OPEN);
            servoLeft.setPosition(HOME_ARM);
            servoRight.setPosition(HOME_ARM);
            servoRotation.setPosition(ROTATE_PICK);

            ElevateLeft.setTargetPosition(0);
            ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ElevateLeft.setPower(0.5);
            ElevateRight.setTargetPosition(0);
            ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ElevateRight.setPower(0.5);

            SlideRight.setTargetPosition(0);
            SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SlideRight.setPower(0.2);

            SlideLeft.setTargetPosition(0);
            SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SlideLeft.setPower(0.2);
        }
//        waitForStart();
        while (opModeIsActive()) {
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

            if (gamepad1.back) {                    // elevate at start position
                servoLeft.setPosition(HOME_ARM);
                servoRight.setPosition(HOME_ARM);
                servoRotation.setPosition(ROTATE_PICK);

                SlideRight.setTargetPosition(0);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(0.6);

                SlideLeft.setTargetPosition(0);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setPower(0.6);

                ElevateLeft.setTargetPosition(0);
                ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateLeft.setPower(0.8);
                ElevateRight.setTargetPosition(0);
                ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateRight.setPower(0.8);


            }

            //************************ CONE PICKUP AT INIT *******************************

            if (gamepad1.left_bumper) {      // cone pickup

                SlideRight.setTargetPosition(MIN);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(0.6);

                SlideLeft.setTargetPosition(MIN);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setPower(0.6);

                servoGripper.setPosition(GRIPPER_OPEN);
                sleep(500);
                servoLeft.setPosition(PICK_ARM_LEFT);
                servoRight.setPosition(PICK_ARM_RIGHT);

                servoRotation.setPosition(ROTATE_PICK);
                SlideRight.setTargetPosition(TEST);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(0.6);

                SlideLeft.setTargetPosition(TEST);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setPower(0.6);
            }

            //************************ CONE TRANSFER CYCLE *******************************
             else if (gamepad1.right_bumper) {           // cone transfer
                SlideRight.setTargetPosition(MAX);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(0.7);

                SlideLeft.setTargetPosition(MAX);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setPower(0.7);

                sleep(100);

                servoGripper.setPosition(GRIPPER_CLOSE);
                sleep(200);
                servoLeft.setPosition(HOME_ARM);
                servoRight.setPosition(HOME_ARM);

                SlideRight.setTargetPosition(0);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(0.95);

                SlideLeft.setTargetPosition(0);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setPower(0.95);

                sleep(900);
                servoRotation.setPosition(ROTATE_DROP);
                SlideRight.setTargetPosition(270);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(0.95);

                SlideLeft.setTargetPosition(270);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setPower(0.95);

                sleep(300);

                servoLeft.setPosition(DROP_ARM_LEFT);
                servoRight.setPosition(DROP_ARM_RIGHT);

                sleep(800);

                servoGripper.setPosition(GRIPPER_OPEN);
                sleep(200);
                servoLeft.setPosition(HOME_ARM);
                servoRight.setPosition(HOME_ARM);
                sleep(200);
                SlideRight.setTargetPosition(0);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(0.6);

                SlideLeft.setTargetPosition(0);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setPower(0.6);
                servoRotation.setPosition(ROTATE_PICK);
            }

             //////////////////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////

            else if (gamepad1.b) {           // storage mode to drop
                servoRotation.setPosition(ROTATE_DROP);

                SlideRight.setTargetPosition(270);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(0.95);

                SlideLeft.setTargetPosition(270);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setPower(0.95);

                sleep(200);

                servoLeft.setPosition(DROP_ARM_LEFT);
                servoRight.setPosition(DROP_ARM_RIGHT);

                sleep(900);

                servoGripper.setPosition(GRIPPER_OPEN);
                sleep(200);
                servoLeft.setPosition(HOME_ARM);
                servoRight.setPosition(HOME_ARM);
                sleep(200);
                SlideRight.setTargetPosition(0);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(0.6);

                SlideLeft.setTargetPosition(0);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setPower(0.6);
                servoRotation.setPosition(ROTATE_PICK);
            }

            ////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////



            //************************ UP DOWN BUTTONS *******************************

            else if (gamepad1.a) {

                ElevateRight.setTargetPosition(LOW);
                ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateRight.setPower(0.4);

                ElevateLeft.setTargetPosition(LOW);
                ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateLeft.setPower(0.4);
            }
            else if (gamepad1.y) {

                ElevateRight.setTargetPosition(MID);
                ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateRight.setPower(0.8);

                ElevateLeft.setTargetPosition(MID);
                ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateLeft.setPower(0.8);
            }
            else if (gamepad1.dpad_up) {

                ElevateRight.setTargetPosition(HIGH);
                ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateRight.setPower(1);

                ElevateLeft.setTargetPosition(HIGH);
                ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateLeft.setPower(1);
            } else if (gamepad1.dpad_down) {
                ElevateRight.setTargetPosition(0);
                ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateRight.setPower(0.95);

                ElevateLeft.setTargetPosition(0);
                ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateLeft.setPower(0.95);

            }


            //////////////START CYCLE////////////////////////

            else if(gamepad1.start){

                SlideRight.setTargetPosition(MAX);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(0.7);

                SlideLeft.setTargetPosition(MAX);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setPower(0.7);

                sleep(100);
                servoGripper.setPosition(GRIPPER_CLOSE);
                sleep(200);

                servoLeft.setPosition(HOME_ARM);
                servoRight.setPosition(HOME_ARM);

                SlideRight.setTargetPosition(0);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(0.95);

                SlideLeft.setTargetPosition(0);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setPower(0.95);

                sleep(1000);
                servoRotation.setPosition(ROTATE_DROP);
                SlideRight.setTargetPosition(270);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(0.95);

                SlideLeft.setTargetPosition(270);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setPower(0.95);

                sleep(300);

                servoLeft.setPosition(DROP_ARM_LEFT);
                servoRight.setPosition(DROP_ARM_RIGHT);

                sleep(600);

                servoGripper.setPosition(GRIPPER_OPEN);
                sleep(200);
                servoLeft.setPosition(PICK_ARM_LEFT);
                servoRight.setPosition(PICK_ARM_RIGHT);
                sleep(200);
                ElevateRight.setTargetPosition(HIGH);
                ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateRight.setPower(0.8);

                ElevateLeft.setTargetPosition(HIGH);
                ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateLeft.setPower(0.8);

                servoRotation.setPosition(ROTATE_PICK);

                sleep(1000);

                ElevateLeft.setTargetPosition(0);
                ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateLeft.setPower(1);
                ElevateRight.setTargetPosition(0);
                ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateRight.setPower(1);

                SlideRight.setTargetPosition(TEST);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(0.8);

                SlideLeft.setTargetPosition(TEST);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setPower(0.8);

            }

            else if(gamepad1.x){

                servoLeft.setPosition(GROUND_ARM_LEFT);
                servoRight.setPosition(GROUND_ARM_RIGHT);

                SlideRight.setTargetPosition(0);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(0.6);

                SlideLeft.setTargetPosition(0);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setPower(0.6);


                ///////////////////MID CYCLE///////////////////////
//                SlideRight.setTargetPosition(MAX);
//                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideRight.setPower(0.7);
//
//                SlideLeft.setTargetPosition(MAX);
//                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideLeft.setPower(0.7);
//
//                sleep(100);
//                servoGripper.setPosition(GRIPPER_CLOSE);
//                sleep(200);
//                servoRotation.setPosition(ROTATE_DROP);
//
//                servoLeft.setPosition(HOME_ARM);
//                servoRight.setPosition(HOME_ARM);
//
//                SlideRight.setTargetPosition(0);
//                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideRight.setPower(0.95);
//
//                SlideLeft.setTargetPosition(0);
//                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideLeft.setPower(0.95);
//
//                sleep(900);
//
//                SlideRight.setTargetPosition(270);
//                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideRight.setPower(0.95);
//
//                SlideLeft.setTargetPosition(270);
//                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideLeft.setPower(0.95);
//
//                sleep(300);
//
//                servoLeft.setPosition(DROP_ARM_LEFT);
//                servoRight.setPosition(DROP_ARM_RIGHT);
//
//                sleep(800);
//
//                servoGripper.setPosition(GRIPPER_OPEN);
//                sleep(200);
//                servoLeft.setPosition(PICK_ARM_LEFT);
//                servoRight.setPosition(PICK_ARM_RIGHT);
//                sleep(200);
//                ElevateRight.setTargetPosition(MID);
//                ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ElevateRight.setPower(1);
//
//                ElevateLeft.setTargetPosition(MID);
//                ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ElevateLeft.setPower(1);
//
//                servoRotation.setPosition(ROTATE_PICK);
//
//                sleep(1000);
//
//                ElevateLeft.setTargetPosition(0);
//                ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ElevateLeft.setPower(1);
//                ElevateRight.setTargetPosition(0);
//                ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ElevateRight.setPower(1);
//
//                SlideRight.setTargetPosition(TEST);
//                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideRight.setPower(0.6);
//
//                SlideLeft.setTargetPosition(TEST);
//                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideLeft.setPower(0.6);

            }
            else if(gamepad1.dpad_right){
                servoGripper.setPosition(GRIPPER_CLOSE);
                sleep(300);
                servoLeft.setPosition(HOME_ARM);
                servoRight.setPosition(HOME_ARM);

                SlideRight.setTargetPosition(0);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(0.6);

                SlideLeft.setTargetPosition(0);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setPower(0.6);
            }
            else if(gamepad1.dpad_left){
                servoLeft.setPosition(0.96);
                servoRight.setPosition(0.04);
            }
            if(gamepad1.right_trigger>0.7)
            {
                servoGripper.setPosition(GRIPPER_OPEN);
            }

            //ManualIncrement
            if(gamepad2.dpad_down){
                ElevateRight.setTargetPosition(ElevateRight.getCurrentPosition()-50);
                ElevateLeft.setTargetPosition(ElevateLeft.getCurrentPosition()-50);
                ElevateRight.setPower(0.5);
                ElevateLeft.setPower(0.5);
            }
            else if(gamepad2.dpad_up){
                ElevateRight.setTargetPosition(ElevateRight.getCurrentPosition()+50);
                ElevateLeft.setTargetPosition(ElevateLeft.getCurrentPosition()+50);
                ElevateRight.setPower(0.5);
                ElevateLeft.setPower(0.5);
            }
            else if(gamepad2.dpad_left){
                SlideRight.setTargetPosition(SlideRight.getCurrentPosition()-50);
                SlideLeft.setTargetPosition(SlideLeft.getCurrentPosition()-50);
                SlideRight.setPower(0.5);
                SlideLeft.setPower(0.5);
            }
            else if(gamepad2.dpad_right){
                SlideRight.setTargetPosition(SlideRight.getCurrentPosition()+50);
                SlideLeft.setTargetPosition(SlideLeft.getCurrentPosition()+50);
                SlideRight.setPower(0.5);
                SlideLeft.setPower(0.5);
            }

            telemetry.addData("Servo Gripper Position", servoGripper.getPosition());
            telemetry.addData("Servo Left Position", servoLeft.getPosition());
            telemetry.addData("Servo Right Position", servoRight.getPosition());
            telemetry.addData("Servo Rotation Positon", servoRotation.getPosition());
            telemetry.addData("Current ElevateLeft", ElevateLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("ElevateLeft", ElevateLeft.getCurrentPosition());
            telemetry.addData("Current ElevateRight", ElevateRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("ElevateRight", ElevateRight.getCurrentPosition());
            telemetry.addData("Current SlideLeft", SlideLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("SlideLeft", SlideLeft.getCurrentPosition());
            telemetry.addData("Current SlideRight", SlideRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("SlideRight", SlideRight.getCurrentPosition());
            telemetry.update();

        }

    }

}

