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
@Config
@TeleOp(name = "BOOOOM")
public class BOOOOM extends LinearOpMode {
    Servo servoGripper;
    Servo servoLF;
    Servo servoLB;
    Servo servoRF;
    Servo servoRB;
    Servo servoRotate;
    Servo servoActive;
    Servo servoLock;
    DcMotorEx ElevateLeft = null;
    DcMotorEx ElevateRight = null;
    DcMotorEx SlideLeft = null;
    DcMotorEx SlideRight = null;

    //OLD VALUES AT 9:1 elevator ratio
    // public static int LOW = 600;
    //    public static int MID = 1150;
    //    public static int HIGH = 1760;

    // NEW VALUES AT 5:1 elevator ratio
//    public static int LOW = 330;   // 320
//    public static int MID = 700;   //740           // counts value
//    public static int HIGH = 1110; //1160

    //NEW VALUES AT 4:1 elevator ratio
    public static int LOW = 228;   // 320
    public static int MID = 484;   //740           //counts value
    public static int HIGH = 755; //1160

//    //NEW VALUES AT 3:1 elevator ratio
//    public static int LOW = 183;   // 320
//    public static int MID = 387;   //740           //counts value
//    public static int HIGH = 610; //1160


//   //OLD VALUES AT 9:1 slider ratio
//    public static int MAX = 1800;   //
//    public static int TEST = 800; //
//    public static int MAX = 1250;   //
    public static int MAX = 690;
//    public static int TEST = 500; //
    public static int TEST = 275; //
    public static int GRIP = 155;

    public static double GRIPPER_OPEN = 0.5 ;
    public static double GRIPPER_CLOSE = 0.7;
    public static double PICK_ARM_LEFT = 1;
    public static double PICK_ARM_RIGHT = 1-PICK_ARM_LEFT;
    public static double DROP_ARM_LEFT = 0.27;
    public static double DROP_ARM_RIGHT = 1-DROP_ARM_LEFT;

    public static double GROUND_ARM_LEFT = 0.8;
    public static double GROUND_ARM_RIGHT = 0.2;
    public static double HOME_ARM = 0.5;
    public static double ROTATE_PICK = 0.17;
    public static double ROTATE_DROP = 0.82;

    public static double ACTIVE_PICK = 0.88;    // active gripping position
    public static double ACTIVE_STABLE = 0.5;   // stable four bar position
    public static double ACTIVE_LOW = 0.69;

    public static double ACTIVE_DROP = 0.35;

    public static double LOCK = 0;
    public static double UNLOCK = 0.3;

    public static double speed = 1;
    public static double turn = 0.8;


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, telemetry);

        servoGripper = hardwareMap.get(Servo.class, "servoGripper");
        servoLock =  hardwareMap.get(Servo.class, "servoLock");
        servoActive = hardwareMap.get(Servo.class, "servoActive");
        servoLF = hardwareMap.get(Servo.class, "servoLF");
        servoLB = hardwareMap.get(Servo.class, "servoLB");
        servoRF = hardwareMap.get(Servo.class, "servoRF");
        servoRB = hardwareMap.get(Servo.class, "servoRB");
        servoRotate = hardwareMap.get(Servo.class, "servoRotate");
        ElevateLeft = hardwareMap.get(DcMotorEx.class, "ElevateLeft");
        ElevateRight = hardwareMap.get(DcMotorEx.class, "ElevateRight");
        SlideLeft = hardwareMap.get(DcMotorEx.class, "SlideLeft");
        SlideRight = hardwareMap.get(DcMotorEx.class, "SlideRight");


        ElevateLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ElevateRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        SlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElevateLeft.setDirection(DcMotorEx.Direction.REVERSE);
        ElevateRight.setDirection(DcMotorEx.Direction.REVERSE);

        SlideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        ElevateLeft.setTargetPositionTolerance(5);
        ElevateRight.setTargetPositionTolerance(5);
        ElevateRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        ElevateLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        SlideRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        SlideLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        SlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);

//        ElevateLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        ElevateRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        SlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        SlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        while (opModeInInit()) {
            servoLock.setPosition(UNLOCK);
            servoRotate.setPosition(ROTATE_PICK);
            servoGripper.setPosition(GRIPPER_OPEN);
            servoLF.setPosition(HOME_ARM);
            servoLB.setPosition(HOME_ARM);
            servoRF.setPosition(HOME_ARM);
            servoRB.setPosition(HOME_ARM);
            servoActive.setPosition(ACTIVE_STABLE);
//
            ElevateLeft.setTargetPosition(0);
            ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ElevateLeft.setPower(0.3);
            ElevateRight.setTargetPosition(0);
            ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ElevateRight.setPower(0.3);

            SlideRight.setTargetPosition(0);
            SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SlideRight.setPower(0.4);

            SlideLeft.setTargetPosition(0);
            SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SlideLeft.setPower(0.4);
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
//                            -gamepad1.left_stick_y * 1,
//                            -gamepad1.left_stick_x * 1,
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

            if(gamepad1.left_stick_button || gamepad1.right_stick_button){
                drive.setPoseEstimate(startPose);
            }

            if (gamepad1.back) {                    // elevate at start position
                servoLF.setPosition(HOME_ARM);
                servoLB.setPosition(HOME_ARM);
                servoRF.setPosition(HOME_ARM);
                servoRB.setPosition(HOME_ARM);
                servoRotate.setPosition(ROTATE_PICK);
                servoActive.setPosition(ACTIVE_STABLE);
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

            else if (gamepad1.left_bumper) {      // cone pickup
                servoLock.setPosition(UNLOCK);
                servoGripper.setPosition(GRIPPER_OPEN);
                servoActive.setPosition(ACTIVE_PICK);
                servoRotate.setPosition(ROTATE_PICK);
                sleep(300);
                servoLF.setPosition(PICK_ARM_LEFT);
                servoLB.setPosition(PICK_ARM_LEFT);
                servoRF.setPosition(PICK_ARM_RIGHT);
                servoRB.setPosition(PICK_ARM_RIGHT);
                sleep(300);
                SlideRight.setTargetPosition(TEST);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(0.7);

                SlideLeft.setTargetPosition(TEST);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setPower(0.7);
            }
            else if (gamepad2.left_bumper) {      // cone pickup

                servoLock.setPosition(UNLOCK);
                servoGripper.setPosition(GRIPPER_OPEN);
                servoActive.setPosition(ACTIVE_PICK);
                servoRotate.setPosition(ROTATE_PICK);
                sleep(200);

                servoLF.setPosition(PICK_ARM_LEFT);
                servoLB.setPosition(PICK_ARM_LEFT);
                servoRF.setPosition(PICK_ARM_RIGHT);
                servoRB.setPosition(PICK_ARM_RIGHT);

                SlideRight.setTargetPosition(MAX);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(1);

                SlideLeft.setTargetPosition(MAX);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setPower(1);
            }
            else if (gamepad2.right_bumper) {           // cone transfer

                SlideRight.setTargetPosition(MAX+50);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(1);

                SlideLeft.setTargetPosition(MAX+50);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setPower(1);
                sleep(100);
                servoGripper.setPosition(GRIPPER_CLOSE);
                sleep(200);

                servoLF.setPosition(HOME_ARM);
                servoLB.setPosition(HOME_ARM);
                servoRF.setPosition(HOME_ARM);
                servoRB.setPosition(HOME_ARM);
                servoActive.setPosition(ACTIVE_STABLE);

                SlideRight.setTargetPosition(0);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(1);

                SlideLeft.setTargetPosition(0);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setPower(1);


                sleep(300);
                servoRotate.setPosition(ROTATE_DROP);
//                sleep(300);

                servoActive.setPosition(ACTIVE_DROP);
                sleep(300);
                servoLF.setPosition(DROP_ARM_LEFT);
                servoLB.setPosition(DROP_ARM_LEFT);
                servoRF.setPosition(DROP_ARM_RIGHT);
                servoRB.setPosition(DROP_ARM_RIGHT);

                sleep(150);

                servoGripper.setPosition(GRIPPER_OPEN);
                sleep(100);
                servoLock.setPosition(LOCK);
                servoLF.setPosition(HOME_ARM);
                servoLB.setPosition(HOME_ARM);
                servoRF.setPosition(HOME_ARM);
                servoRB.setPosition(HOME_ARM);
                servoRotate.setPosition(ROTATE_PICK);
                sleep(200);

                servoActive.setPosition(ACTIVE_STABLE);

            }

//            //************************ CONE TRANSFER CYCLE *******************************
            else if (gamepad1.right_bumper) {           // cone transfer
                SlideRight.setTargetPosition(TEST+100);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(0.9);

                SlideLeft.setTargetPosition(TEST+100);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setPower(0.9);
                sleep(100);
                servoGripper.setPosition(GRIPPER_CLOSE);
                sleep(200);

                servoLF.setPosition(HOME_ARM);
                servoLB.setPosition(HOME_ARM);
                servoRF.setPosition(HOME_ARM);
                servoRB.setPosition(HOME_ARM);
                servoActive.setPosition(ACTIVE_STABLE);

                SlideRight.setTargetPosition(0);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(0.9);

                SlideLeft.setTargetPosition(0);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setPower(0.9);


                sleep(200);
                servoRotate.setPosition(ROTATE_DROP);
//                sleep(300);

                servoActive.setPosition(ACTIVE_DROP);
                sleep(400);
                servoLF.setPosition(DROP_ARM_LEFT);
                servoLB.setPosition(DROP_ARM_LEFT);
                servoRF.setPosition(DROP_ARM_RIGHT);
                servoRB.setPosition(DROP_ARM_RIGHT);

                sleep(500);

                servoGripper.setPosition(GRIPPER_OPEN);
                sleep(100);
                servoLock.setPosition(LOCK);
                servoLF.setPosition(HOME_ARM);
                servoLB.setPosition(HOME_ARM);
                servoRF.setPosition(HOME_ARM);
                servoRB.setPosition(HOME_ARM);
                servoRotate.setPosition(ROTATE_PICK);
                sleep(200);

                servoActive.setPosition(ACTIVE_STABLE);
            }
            else if (gamepad1.b) {           // cone transfer

                servoGripper.setPosition(GRIPPER_CLOSE);
                sleep(200);
                servoRotate.setPosition(ROTATE_DROP);
                servoActive.setPosition(ACTIVE_DROP);

                servoLF.setPosition(DROP_ARM_LEFT);
                servoLB.setPosition(DROP_ARM_LEFT);
                servoRF.setPosition(DROP_ARM_RIGHT);
                servoRB.setPosition(DROP_ARM_RIGHT);

                sleep(800);

                servoGripper.setPosition(GRIPPER_OPEN);
                sleep(200);
                servoLF.setPosition(HOME_ARM);
                servoLB.setPosition(HOME_ARM);
                servoRF.setPosition(HOME_ARM);
                servoRB.setPosition(HOME_ARM);
                sleep(100);
                servoLock.setPosition(LOCK);
                servoRotate.setPosition(ROTATE_PICK);
                servoActive.setPosition(ACTIVE_STABLE);
            }

            //////////////////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////


            ////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////



            //************************ UP DOWN BUTTONS *******************************

            else if (gamepad1.a) {

                ElevateRight.setTargetPosition(LOW);
                ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateRight.setPower(0.6);

                ElevateLeft.setTargetPosition(LOW);
                ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateLeft.setPower(0.6);
            }
            else if (gamepad1.y) {

                ElevateRight.setTargetPosition(MID);
                ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateRight.setPower(1);

                ElevateLeft.setTargetPosition(MID);
                ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateLeft.setPower(1);
            }

            else if (gamepad1.dpad_up) {

                ElevateRight.setTargetPosition(HIGH);
                ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateRight.setPower(1);

                ElevateLeft.setTargetPosition(HIGH);
                ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateLeft.setPower(1);

            }
            else if (gamepad1.dpad_down) {
                servoLock.setPosition(UNLOCK);

                ElevateRight.setTargetPosition(0);
                ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateRight.setPower(0.9);

                ElevateLeft.setTargetPosition(0);
                ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateLeft.setPower(0.9);


            }
            //small start cycle for fun

            else if (gamepad2.start) {
                servoGripper.setPosition(GRIPPER_CLOSE);
                sleep(200);
                servoRotate.setPosition(ROTATE_DROP);
                servoActive.setPosition(ACTIVE_DROP);

                servoLF.setPosition(DROP_ARM_LEFT);
                servoLB.setPosition(DROP_ARM_LEFT);
                servoRF.setPosition(DROP_ARM_RIGHT);
                servoRB.setPosition(DROP_ARM_RIGHT);

                sleep(800);

                servoGripper.setPosition(GRIPPER_OPEN);
                sleep(200);
                servoLF.setPosition(HOME_ARM);
                servoLB.setPosition(HOME_ARM);
                servoRF.setPosition(HOME_ARM);
                servoRB.setPosition(HOME_ARM);
                sleep(100);
                servoLock.setPosition(LOCK);
                servoRotate.setPosition(ROTATE_PICK);
                servoActive.setPosition(ACTIVE_STABLE);
                ElevateRight.setTargetPosition(HIGH);
                ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateRight.setPower(1);

                ElevateLeft.setTargetPosition(HIGH);
                ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateLeft.setPower(1);
                sleep(700);

                servoActive.setPosition(ACTIVE_PICK);
                servoLF.setPosition(PICK_ARM_LEFT);
                servoLB.setPosition(PICK_ARM_LEFT);
                servoRF.setPosition(PICK_ARM_RIGHT);
                servoRB.setPosition(PICK_ARM_RIGHT);

                //sleep(200);
                servoLock.setPosition(UNLOCK);

                ElevateRight.setTargetPosition(0);
                ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateRight.setPower(0.9);

                ElevateLeft.setTargetPosition(0);
                ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateLeft.setPower(0.9);


            }


            ///////////////////REAL CYCLEE////////////////////////

            else if (gamepad1.start) {
                SlideRight.setTargetPosition(TEST+100);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(0.9);

                SlideLeft.setTargetPosition(TEST+100);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setPower(0.9);
                sleep(100);

                servoGripper.setPosition(GRIPPER_CLOSE);
                sleep(200);

                servoLF.setPosition(HOME_ARM);
                servoLB.setPosition(HOME_ARM);
                servoRF.setPosition(HOME_ARM);
                servoRB.setPosition(HOME_ARM);
                servoActive.setPosition(ACTIVE_STABLE);

                SlideRight.setTargetPosition(0);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(1);

                SlideLeft.setTargetPosition(0);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setPower(1);


                sleep(200);
                servoRotate.setPosition(ROTATE_DROP);
//                sleep(300);

                servoActive.setPosition(ACTIVE_DROP);
                sleep(200);
                servoLF.setPosition(DROP_ARM_LEFT);
                servoLB.setPosition(DROP_ARM_LEFT);
                servoRF.setPosition(DROP_ARM_RIGHT);
                servoRB.setPosition(DROP_ARM_RIGHT);

                sleep(300);

                servoGripper.setPosition(GRIPPER_OPEN);
                sleep(200);
                servoLock.setPosition(LOCK);
                servoLF.setPosition(PICK_ARM_LEFT);
                servoLB.setPosition(PICK_ARM_LEFT);
                servoRF.setPosition(PICK_ARM_RIGHT);
                servoRB.setPosition(PICK_ARM_RIGHT);
                servoRotate.setPosition(ROTATE_PICK);
                servoActive.setPosition(ACTIVE_PICK);
                sleep(200);

                ElevateRight.setTargetPosition(HIGH);
                ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateRight.setPower(1);

                ElevateLeft.setTargetPosition(HIGH);
                ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateLeft.setPower(1);
                sleep(700);

                SlideRight.setTargetPosition(TEST);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(0.7);

                SlideLeft.setTargetPosition(TEST);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setPower(0.7);

                //sleep(200);
                servoLock.setPosition(UNLOCK);

                ElevateRight.setTargetPosition(0);
                ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateRight.setPower(1);

                ElevateLeft.setTargetPosition(0);
                ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateLeft.setPower(1);

            }


            else if(gamepad1.dpad_right){
                servoRotate.setPosition(ROTATE_PICK);
                servoGripper.setPosition(GRIPPER_CLOSE);
                sleep(300);
                servoActive.setPosition(ACTIVE_LOW);
                servoLF.setPosition(HOME_ARM);
                servoLB.setPosition(HOME_ARM);
                servoRF.setPosition(HOME_ARM);
                servoRB.setPosition(HOME_ARM);

                SlideRight.setTargetPosition(0);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(0.7);

                SlideLeft.setTargetPosition(0);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setPower(0.7);
            }
            else if(gamepad1.dpad_left){
                servoRotate.setPosition(ROTATE_PICK);
                servoGripper.setPosition(GRIPPER_OPEN);
                servoActive.setPosition(ACTIVE_PICK);
                sleep(300);
                servoLF.setPosition(PICK_ARM_LEFT);
                servoLB.setPosition(PICK_ARM_LEFT);
                servoRF.setPosition(PICK_ARM_RIGHT);
                servoRB.setPosition(PICK_ARM_RIGHT);
            }
            else if(gamepad1.x){
                servoRotate.setPosition(ROTATE_PICK);
                servoActive.setPosition(ACTIVE_PICK);
                sleep(300);
                servoLF.setPosition(PICK_ARM_LEFT);
                servoLB.setPosition(PICK_ARM_LEFT);
                servoRF.setPosition(PICK_ARM_RIGHT);
                servoRB.setPosition(PICK_ARM_RIGHT);
            }

            if(gamepad1.right_trigger>0.7)
            {
                servoGripper.setPosition(GRIPPER_OPEN);
            }
            else if(gamepad2.right_trigger>0.7)
            {
                servoGripper.setPosition(GRIPPER_OPEN);
            }
            else if(gamepad2.left_trigger>0.7)
            {
                servoGripper.setPosition(GRIPPER_CLOSE);
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

            else if(gamepad2.y){
                servoActive.setPosition(ACTIVE_PICK);
            }
            else if(gamepad2.a){
                servoActive.setPosition(ACTIVE_DROP);
            }
            else if(gamepad2.x){
                servoLock.setPosition(LOCK);
            }
            else if(gamepad2.b){
                servoLock.setPosition(UNLOCK);
            }
//            if(gamepad2.left_trigger>0.9)
//            {
//                SlideRight.setTargetPosition(1500);
//                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideRight.setPower(0.8);
//
//                SlideLeft.setTargetPosition(1500);
//                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideLeft.setPower(0.8);
//            } else if (gamepad2.left_trigger>0.8) {
//                SlideRight.setTargetPosition(1200);
//                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideRight.setPower(0.8);
//
//                SlideLeft.setTargetPosition(1200);
//                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideLeft.setPower(0.8);
//            }
//            else if (gamepad2.left_trigger>0.7) {
//                SlideRight.setTargetPosition(900);
//                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideRight.setPower(0.8);
//
//                SlideLeft.setTargetPosition(900);
//                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideLeft.setPower(0.8);
//            }
//            else if (gamepad2.left_trigger>0.6) {
//                SlideRight.setTargetPosition(600);
//                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideRight.setPower(0.8);
//
//                SlideLeft.setTargetPosition(600);
//                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideLeft.setPower(0.8);
//            }
//            else if (gamepad2.left_trigger>0.3) {
//                SlideRight.setTargetPosition(600);
//                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideRight.setPower(0.8);
//
//                SlideLeft.setTargetPosition(600);
//                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideLeft.setPower(0.8);
//            }
//            else if (gamepad2.left_trigger>0.1) {
//                SlideRight.setTargetPosition(100);
//                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideRight.setPower(0.8);
//
//                SlideLeft.setTargetPosition(100);
//                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideLeft.setPower(0.8);
//            }

//            telemetry.addData("Servo Gripper Position", servoGripper.getPosition());
//            telemetry.addData("Servo Left Position", servoLF.getPosition());
//            telemetry.addData("Servo Right Position", servoRF.getPosition());
//            telemetry.addData("Servo Rotation Positon", servoRotate.getPosition());
            telemetry.addData("Current ElevateLeft", ElevateLeft.getCurrent(CurrentUnit.AMPS));

            telemetry.addData("ElevateLeft", ElevateLeft.getCurrentPosition());
            telemetry.addData("Current ElevateRight", ElevateRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("ElevateRight", ElevateRight.getCurrentPosition());
            telemetry.addData("Current SlideLeft", SlideLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("SlideLeft", SlideLeft.getCurrentPosition());
            telemetry.addData("Current SlideRight", SlideRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("SlideRight", SlideRight.getCurrentPosition());
//            telemetry.addData("PowerLeft", ElevateLeft.getPower());
//            telemetry.addData("PowerRight", ElevateRight.getPower());
            telemetry.update();

        }

    }

}

