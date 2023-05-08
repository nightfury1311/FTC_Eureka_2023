package org.firstinspires.ftc.teamcode.drive.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp
public class ServoArmTest extends LinearOpMode {
    Servo servoLeftF;
    Servo servoLeftB;
    Servo servoRightF;
    Servo servoRightB;
    public static double PICK_ARM_LEFT = 0.6;
    public static double PICK_ARM_RIGHT = 0.4;
    public static double DROP_ARM_LEFT = 0.4;
    public static double DROP_ARM_RIGHT = 0.6;
    public static double HOME_ARM = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        servoLeftF = hardwareMap.get(Servo.class, "servoLF");
        servoLeftB = hardwareMap.get(Servo.class, "servoLB");
        servoRightF = hardwareMap.get(Servo.class, "servoRF");
        servoRightB = hardwareMap.get(Servo.class, "servoRB");

        while (opModeInInit()) {
            servoLeftF.setPosition(HOME_ARM);
            servoLeftB.setPosition(HOME_ARM);
            servoRightF.setPosition(HOME_ARM);
            servoRightB.setPosition(HOME_ARM);
        }
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                servoLeftF.setPosition(DROP_ARM_LEFT);
                servoLeftB.setPosition(DROP_ARM_LEFT);
                servoRightF.setPosition(DROP_ARM_RIGHT);
                servoRightB.setPosition(DROP_ARM_RIGHT);
            } else if (gamepad1.dpad_down) {
                servoLeftF.setPosition(PICK_ARM_LEFT);
                servoLeftB.setPosition(PICK_ARM_LEFT);
                servoRightF.setPosition(PICK_ARM_RIGHT);
                servoRightB.setPosition(PICK_ARM_RIGHT);
            } else if (gamepad1.dpad_right) {
                servoLeftF.setPosition(HOME_ARM);
                servoLeftB.setPosition(HOME_ARM);
                servoRightF.setPosition(HOME_ARM);
                servoRightB.setPosition(HOME_ARM);
            }
        }
    }
}
