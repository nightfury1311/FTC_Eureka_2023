package org.firstinspires.ftc.teamcode.drive.opmode;


import static org.firstinspires.ftc.teamcode.util.AngleFunction.angleWrap;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Config
@TeleOp(name = "EurekaTeleOp")
public class eureka_1 extends LinearOpMode {
    public static double kp_gyro = 0.004;
    public static double kd_gyro = 0.005;
    public static double ki_gyro = 0.01;
    double prevError_gyro = 0;

    double previousHeading = 0;
    double integratedHeading = 0;

    private double headingTarget = 0;

    Servo servoGripper;
    Servo servoLeft;
    Servo servoRight;
    Servo servoRotation;
    DcMotorEx ElevateLeft = null;
    DcMotorEx ElevateRight = null;
    DcMotorEx SlideLeft = null;
    DcMotorEx SlideRight = null;

    int posElevate;
    int posSlide;
    double left;
    double right;
    //    Formula & parameters for relation between counts and mm
//    double PulleyDiameter = 30.0;
//    double CPR = 28.0;
//    double GearRatio = 8.4;         // gear ratio from the table
//    double counts = (CPR * GearRatio) / (PulleyDiameter * Math.PI);

//    double highmm = ;               // target mm value
//    double midmm = ;                // target mm value

    int LOW = 900;
    int MID = 2100;                 // counts value
    int HIGH = 3300;
    int MAX = 1150;   // MAX IS 1050 , dont go above this value
    int MIN = 250;

//    double HIGH = counts * highmm;
//    double MID = counts * midmm;

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
//        Arm = hardwareMap.get(DcMotorEx.class, "Arm");

//        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElevateLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElevateRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        SlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElevateLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElevateRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeInInit()) {
            servoLeft.setPosition(0.5);
            servoRight.setPosition(0.5);
            servoRotation.setPosition(0 );
        }
//        waitForStart();

        while (opModeIsActive()) {
            Pose2d poseEstimate = drive.getPoseEstimate();
            double heading = getIntegratedHeading(poseEstimate);

            if (gamepad1.right_stick_x > 0.2) {
                headingTarget -= 2;
            } else if (gamepad1.right_stick_x < -0.2) {
                headingTarget += 2;
            }

            double headingPower = gyroPID(heading, headingTarget);
            headingPower = Range.clip(headingPower, -1.0, 1.0);


            Vector2d input = new Vector2d(-gamepad1.left_stick_y * 0.8, -gamepad1.left_stick_x * 0.8).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), -headingPower));
            drive.update();
            telemetry.addData("Status", "Running");

//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                               -gamepad1.left_stick_y,
//                            -gamepad1.left_stick_x * 0.5,
//                            -gamepad1.right_stick_x * 0.5
////                            gamepad1.right_stick_x+(gamepad1.left_stick_x*0.01)+(-gamepad1.left_stick_y*0.01)
//                    ));
//            drive.update();


            if (gamepad1.dpad_right) {
                posSlide -= 20;

//                SlideRight.setTargetPosition(MAX);
//                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideRight.setPower(0.7);
//
//                SlideLeft.setTargetPosition(MAX);
//                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideLeft.setPower(0.7);
            } else if (gamepad1.dpad_left) {

                posSlide += 20;
//                SlideRight.setTargetPosition(MIN);
//                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideRight.setPower(0.7);
//
//                SlideLeft.setTargetPosition(MIN);
//                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideLeft.setPower(0.7);
//
//                SlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                SlideLeft.setPower(0);
//
//                SlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                SlideRight.setPower(0);
            }
            if (gamepad1.dpad_up) {
                posElevate += 20;
            } else if (gamepad1.dpad_down) {
                posElevate -= 20;
            }
//
//            if (gamepad2.dpad_up) {
//
//                ElevateRight.setTargetPosition(HIGH);
//                ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ElevateRight.setPower(0.95);
//
//                ElevateLeft.setTargetPosition(HIGH);
//                ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ElevateLeft.setPower(0.95);
//
//            } else if (gamepad2.dpad_right) {
//
//                ElevateRight.setTargetPosition(MID);
//                ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ElevateRight.setPower(0.95);
//
//                ElevateLeft.setTargetPosition(MID);
//                ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ElevateLeft.setPower(0.95);
//            } else if (gamepad2.dpad_down) {
//                ElevateRight.setTargetPosition(LOW);
//                ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ElevateRight.setPower(0.95);
//
//                ElevateLeft.setTargetPosition(LOW);
//                ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ElevateLeft.setPower(0.95);
//            } else
            if (gamepad1.back) {                    // elevate at start position

                posElevate = 0;

//                ElevateLeft.setTargetPosition(0);
//                ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ElevateLeft.setPower(0.9);
//                ElevateRight.setTargetPosition(0);
//                ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ElevateRight.setPower(0.9);
            }


//                ElevateLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                ElevateLeft.setPower(0);
//
//                ElevateRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                ElevateRight.setPower(0);
//            }

            //************************ CONE ROTATE *******************************
            if (gamepad1.right_trigger > 0.8) {              // cone drop position
                servoRotation.setPosition(0.65 );
            } else if (gamepad1.left_trigger > 0.8) {       // cone pick position
                servoRotation.setPosition(0);
            }
            //************************ MOTOR ARM *******************************
//            if (gamepad1.dpad_up) {              // arm up
//                Arm.setTargetPosition(400);
//                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                Arm.setPower(0.5);
//            } else if (gamepad1.dpad_down) {    // arm down
//                Arm.setTargetPosition(0);
//                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                Arm.setPower(0.5);
//            }

            //************************ SERVO ARM *******************************

            if (gamepad1.right_bumper) {              // arm up
                servoLeft.setPosition(0.18);
                servoRight.setPosition(0.82);
            } else if (gamepad1.left_bumper) {    // arm down
                servoLeft.setPosition(1);
                servoRight.setPosition(0);
            } else if (gamepad1.start) {         // arm in center position
                servoLeft.setPosition(0.5);
                servoRight.setPosition(0.5);
            }
//                servoLeft.setPosition(0.6);
//                for(left = 0.6; left < 0.8; left+=0.01){
//                    servoLeft.setPosition(left);
//                }
//                servoRight.setPosition(0.4);
//                for(right = 0.4; right > 0.2; right-=0.01){
//                    servoLeft.setPosition(right);
//                }


            //************************ SERVO GRIPPER *******************************

            if (gamepad1.x) {                        //gripper open
                servoGripper.setPosition(0);
            } else if (gamepad1.b) {                 // grippper close
                servoGripper.setPosition(0.4);
            }

            //************************ CONE TRANSFER *******************************

//            if (gamepad2.right_bumper){           // cone transfer
//                servoLeft.setPosition(0.18);
//                servoRight.setPosition(0.82);
//                servoRotation.setPosition(1);
////
//                SlideRight.setTargetPosition(MIN);
//                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideRight.setPower(0.7);
//
//                SlideLeft.setTargetPosition(MIN);
//                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideLeft.setPower(0.7);
////
//           }
//            if (gamepad1.y) {      // cone pickup
//                servoRotation.setPosition(0);
//                servoLeft.setPosition(1);
//                servoRight.setPosition(0);
//                servoGripper.setPosition(0.4);
//
//                SlideRight.setTargetPosition(MAX);
//                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideRight.setPower(0.7);
//
//                SlideLeft.setTargetPosition(MAX);
//                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideLeft.setPower(0.7);
//            }
            //************************ MIDDLE CYCLE *******************************
//            if (gamepad1.right_bumper) {           // cone transfer
//                ElevateLeft.setTargetPosition(0);
//                ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ElevateLeft.setPower(0.95);
//                ElevateRight.setTargetPosition(0);
//                ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ElevateRight.setPower(0.95);
//                sleep(200);
//                servoGripper.setPosition(0);
//                sleep(200);
//                servoLeft.setPosition(0.16);
//                servoRight.setPosition(0.84);
//                servoRotation.setPosition(1);
////
//                SlideRight.setTargetPosition(MIN);
//                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideRight.setPower(0.7);
//
//                SlideLeft.setTargetPosition(MIN);
//                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideLeft.setPower(0.7);
//            } else if (gamepad1.left_bumper) {      // cone pickup
//                servoGripper.setPosition(0.4);
//                sleep(300);
//                servoLeft.setPosition(1);
//                servoRight.setPosition(0);
//                servoGripper.setPosition(0.4);
//                sleep(100);
//                servoRotation.setPosition(0);
//                ElevateRight.setTargetPosition(MID);
//                ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ElevateRight.setPower(0.95);
//
//                ElevateLeft.setTargetPosition(MID);
//                ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ElevateLeft.setPower(0.95);
//
//                SlideRight.setTargetPosition(MAX);
//                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideRight.setPower(0.7);
//
//                SlideLeft.setTargetPosition(MAX);
//                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideLeft.setPower(0.7);
//            }

            ElevateRight.setTargetPosition(posElevate);
            ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ElevateRight.setPower(0.9);

            ElevateLeft.setTargetPosition(posElevate);
            ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ElevateLeft.setPower(0.9);

            SlideRight.setTargetPosition(posSlide);
            SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SlideRight.setPower(0.5);

            SlideLeft.setTargetPosition(posSlide);
            SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SlideLeft.setPower(0.5);


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
//            telemetry.addData("Arm Current", Arm.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Arm Position", Arm.getCurrentPosition());
            telemetry.update();
        }
    }
    private double gyroPID(double current, double target){
        double error = current - target;
        double pError = error;
        double dError = error - prevError_gyro;
        double Ierror = error + prevError_gyro;

        prevError_gyro = error;

        return pError * kp_gyro + dError * kd_gyro + Ierror * ki_gyro;
    }


    private double getIntegratedHeading(Pose2d poseEstimate) {
        double currentHeading = Math.toDegrees(angleWrap(poseEstimate.getHeading()));
        double deltaHeading = currentHeading - previousHeading;

        if(deltaHeading < -180){
            deltaHeading +=360;
        }
        else if(deltaHeading >= 180){
            deltaHeading -= 360;
        }

        integratedHeading += deltaHeading;
        previousHeading = currentHeading;

        return integratedHeading;
    }
}
