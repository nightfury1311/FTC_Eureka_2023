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
@Disabled
@Config
@TeleOp(name = "ElevatorTest")
public class ElevatorTest extends LinearOpMode {
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

    public static int LOW = 183;   // 320
    public static int MID = 387;   //740           //counts value
    public static int HIGH = 610; //1160


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
    public static double DROP_ARM_LEFT = 0.25;
    public static double DROP_ARM_RIGHT = 1-DROP_ARM_LEFT;

    public static double GROUND_ARM_LEFT = 0.8;
    public static double GROUND_ARM_RIGHT = 0.2;
    public static double HOME_ARM = 0.5;
    public static double ROTATE_PICK = 0.17;
    public static double ROTATE_DROP = 0.82;

    public static double ACTIVE_PICK = 0.88;    // active gripping position
    public static double ACTIVE_STABLE = 0.5;   // stable four bar position
    public static double ACTIVE_LOW = 0.69;

    public static double ACTIVE_DROP = 0.3;

    public static double LOCK = 0;
    public static double UNLOCK = 0.3;

    public static double speed = 0.9;
    public static double turn = 0.6;


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



        ElevateLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElevateRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ElevateLeft.setDirection(DcMotorEx.Direction.REVERSE);
        ElevateRight.setDirection(DcMotorEx.Direction.REVERSE);




        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        }
//        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            Pose2d poseEstimate = drive.getPoseEstimate();
            Vector2d input = new Vector2d(Math.pow(-gamepad1.left_stick_y,3), Math.pow(-gamepad1.left_stick_x,3)).rotated(-poseEstimate.getHeading());

//            drive.setWeightedDrivePower(new Pose2d(input.getX() * speed, input.getY() * speed, -gamepad1.right_stick_x * turn));
//            drive.update();


            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * 1,
                            -gamepad1.left_stick_x * 1,
                            -gamepad1.right_stick_x * 0.7
//
                    ));
            drive.update();

            if(gamepad1.left_trigger>0.7 )    // slow mode
            {
                speed = 0.4;
                turn = 0.2;
            } else{
                speed = 0.9;
                turn = 0.6;
            }


            //************************ UP DOWN BUTTONS *******************************


           if (gamepad1.dpad_up) {

                ElevateLeft.setTargetPosition(HIGH);
                ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateLeft.setPower(1);

                ElevateRight.setTargetPosition(HIGH);
                ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateRight.setPower(1);

            }
            else if (gamepad1.y) {

                ElevateLeft.setTargetPosition(MID);
                ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateLeft.setPower(1);

                ElevateRight.setTargetPosition(MID);
                ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateRight.setPower(1);

            }
            else if (gamepad1.a) {

                ElevateLeft.setTargetPosition(LOW);
                ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateLeft.setPower(1);

                ElevateRight.setTargetPosition(LOW);
                ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ElevateRight.setPower(1);
            }
            else if (gamepad1.dpad_down) {
               ElevateLeft.setTargetPosition(0);
               ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               ElevateLeft.setPower(1);

                ElevateRight.setTargetPosition(0);
                ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               ElevateRight.setPower(1);
            }

            telemetry.addData("Current ElevateLeft", ElevateLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("ElevateLeft", ElevateLeft.getCurrentPosition());
            telemetry.addData("Current ElevateRight", ElevateRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("ElevateRight", ElevateRight.getCurrentPosition());
            telemetry.addData("Power ElevateLeft", ElevateLeft.getPower());
            telemetry.addData("Power ElevateRight", ElevateRight.getPower());

            telemetry.update();

        }

    }

}

