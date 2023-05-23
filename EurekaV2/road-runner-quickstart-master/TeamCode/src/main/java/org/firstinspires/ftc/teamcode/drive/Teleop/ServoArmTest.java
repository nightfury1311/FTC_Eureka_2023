package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp
public class servoarmtest extends LinearOpMode {
    Servo servoLeft;
    Servo servoRight;
    public static double PICK_ARM_LEFT = 1;
    public static double PICK_ARM_RIGHT = 0;
    public static double DROP_ARM_LEFT = 0.2;
    public static double DROP_ARM_RIGHT = 0.8;
    public static double HOME_ARM = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");
        while (opModeInInit()) {
            servoLeft.setPosition(HOME_ARM);
            servoRight.setPosition(HOME_ARM);
        }
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                servoLeft.setPosition(DROP_ARM_LEFT);
                servoRight.setPosition(DROP_ARM_RIGHT);
            } else if (gamepad1.dpad_down) {
                servoLeft.setPosition(PICK_ARM_LEFT);
                servoRight.setPosition(PICK_ARM_RIGHT);
            } else if (gamepad1.dpad_right) {
                servoLeft.setPosition(PICK_ARM_LEFT);
                servoRight.setPosition(PICK_ARM_RIGHT);
            }
        }
    }
}
