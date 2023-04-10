package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@Disabled
@TeleOp
public class SensorSlider extends LinearOpMode {
    DcMotorEx SlideLeft = null;
    DcMotorEx SlideRight = null;
    Servo servoLeft;
    Servo servoRight;
    TouchSensor HomeMag;
    TouchSensor DropMag;
    public static int HOME = 0;
    public static int MIN = 230;
    public static int TEST = 100;
    public static double HOME_ARM = 0.5;
    int posSlide;
    boolean testflag = false;

    @Override
    public void runOpMode() throws InterruptedException {

        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");
        SlideLeft = hardwareMap.get(DcMotorEx.class, "SlideLeft");
        SlideRight = hardwareMap.get(DcMotorEx.class, "SlideRight");
        DropMag = hardwareMap.get(TouchSensor.class, "DropMag");
        HomeMag = hardwareMap.get(TouchSensor.class, "HomeMag");

        SlideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        SlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        SlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeInInit()) {
            servoLeft.setPosition(HOME_ARM);
            servoRight.setPosition(HOME_ARM);
        }

        while (opModeIsActive()) {

            if(gamepad1.back){
                while(!HomeMag.isPressed() && opModeIsActive()){
                    SlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    SlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    SlideRight.setPower(-0.8);
                    SlideLeft.setPower(-0.8);
                }
                SlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                SlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                SlideRight.setTargetPosition(0);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(0.7);
                SlideLeft.setTargetPosition(0);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setPower(0.7);
            }

            if(gamepad1.start && SlideRight.getCurrentPosition() > 150){
                while(!DropMag.isPressed() && opModeIsActive()){
                    SlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    SlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    telemetry.addData("dropmag", DropMag.isPressed());
                    telemetry.update();
                    SlideRight.setPower(-0.8);
                    SlideLeft.setPower(-0.8);
                }
                int currentEncoderLeft = SlideLeft.getCurrentPosition();
                int  currentEncoderRight = SlideRight.getCurrentPosition();

                SlideRight.setTargetPosition(currentEncoderRight + 5);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setPower(0.7);
                SlideLeft.setTargetPosition(currentEncoderLeft + 5);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideLeft.setPower(0.7);
            }

            if(gamepad1.right_bumper){
                SlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                SlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                SlideRight.setPower(0.6);
                SlideLeft.setPower(0.6);
            }

            else if(gamepad1.left_bumper){
                SlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                SlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                SlideRight.setPower(-0.6);
                SlideLeft.setPower(-0.6);
            }
            else{
                if(SlideLeft.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                    SlideLeft.setPower(0);
                    SlideRight.setPower(0);
                }
            }

//            if (gamepad1.dpad_right) {
//                SlideRight.setTargetPosition(TEST);
//                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideRight.setPower(0.6);
//
//                SlideLeft.setTargetPosition(TEST);
//                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideLeft.setPower(0.6);
//            } else if (gamepad1.dpad_left) {
//                SlideRight.setTargetPosition(MIN);
//                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideRight.setPower(0.6);
//
//                SlideLeft.setTargetPosition(MIN);
//                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                SlideLeft.setPower(0.6);
//            } else if(gamepad1.back) {
//
//                if (!HomeMag.isPressed()) {
//                    SlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    SlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    SlideLeft.setPower(-0.3);
//                    SlideRight.setPower(-0.3);
//
//                    if (HomeMag.isPressed()){
//                    SlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    SlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    SlideLeft.setPower(0);
//                    SlideRight.setPower(0);
//                }
//            }
//            }

            telemetry.addData("HomeMag", HomeMag.isPressed());
            telemetry.addData("DropMag", DropMag.isPressed());
            telemetry.addData("Current SlideLeft", SlideLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("SlideLeft", SlideLeft.getCurrentPosition());
            telemetry.addData("Current SlideRight", SlideRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("SlideRight", SlideRight.getCurrentPosition());
            telemetry.update();
        }
    }
}
