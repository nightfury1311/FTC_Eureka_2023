package org.firstinspires.ftc.teamcode.drive.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
//@Disabled
@Config
@TeleOp
public class ServoArmTest extends LinearOpMode {
    Servo servoGripper;
    Servo servoLF;
    Servo servoLB;
    Servo servoRF;
    Servo servoRB;
    Servo servoRotate;
    Servo servoActive;
    Servo servoLock;
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
    public static double UNLOCK = 0.28;

    @Override
    public void runOpMode() throws InterruptedException {
        servoGripper = hardwareMap.get(Servo.class, "servoGripper");
        servoLock =  hardwareMap.get(Servo.class, "servoLock");
        servoActive = hardwareMap.get(Servo.class, "servoActive");
        servoLF = hardwareMap.get(Servo.class, "servoLF");
        servoLB = hardwareMap.get(Servo.class, "servoLB");
        servoRF = hardwareMap.get(Servo.class, "servoRF");
        servoRB = hardwareMap.get(Servo.class, "servoRB");
        servoRotate = hardwareMap.get(Servo.class, "servoRotate");
        while (opModeInInit()) {
            servoLock.setPosition(UNLOCK);
            servoRotate.setPosition(ROTATE_PICK);
            servoGripper.setPosition(GRIPPER_OPEN);
            servoLF.setPosition(HOME_ARM);
            servoLB.setPosition(HOME_ARM);
            servoRF.setPosition(HOME_ARM);
            servoRB.setPosition(HOME_ARM);
            servoActive.setPosition(ACTIVE_STABLE);
        }
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                servoLF.setPosition(DROP_ARM_LEFT);
                servoLB.setPosition(DROP_ARM_LEFT);
                servoRF.setPosition(DROP_ARM_RIGHT);
                servoRB.setPosition(DROP_ARM_RIGHT);
            } else if (gamepad1.dpad_down) {
                servoLF.setPosition(PICK_ARM_LEFT);
                servoLB.setPosition(PICK_ARM_LEFT);
                servoRF.setPosition(PICK_ARM_RIGHT);
                servoRB.setPosition(PICK_ARM_RIGHT);
            } else if (gamepad1.dpad_right) {
                servoLF.setPosition(HOME_ARM);
                servoLB.setPosition(HOME_ARM);
                servoRF.setPosition(HOME_ARM);
                servoRB.setPosition(HOME_ARM);
            } else if(gamepad1.b){
                servoRotate.setPosition(ROTATE_DROP);
            }
            else if(gamepad1.x){
                servoRotate.setPosition(ROTATE_PICK);
            } else if (gamepad1.left_bumper) {
                servoActive.setPosition(ACTIVE_PICK);
            }
            else if (gamepad1.right_bumper) {
                servoActive.setPosition(ACTIVE_DROP);
            }
            else if (gamepad1.back) {

                servoRotate.setPosition(ROTATE_PICK);
                servoGripper.setPosition(GRIPPER_OPEN);
                sleep(300);
                servoActive.setPosition(ACTIVE_PICK);
                servoLF.setPosition(PICK_ARM_LEFT);
                servoLB.setPosition(PICK_ARM_LEFT);
                servoRF.setPosition(PICK_ARM_RIGHT);
                servoRB.setPosition(PICK_ARM_RIGHT);

            }
            else if (gamepad1.right_trigger>0.7) {
                servoGripper.setPosition(GRIPPER_OPEN);
            }
            else if (gamepad1.left_trigger>0.7) {
                servoGripper.setPosition(GRIPPER_CLOSE);
            }

            else if (gamepad1.start) {
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
                sleep(200);
                servoLock.setPosition(LOCK);
                sleep(200);
                servoRotate.setPosition(ROTATE_PICK);
                servoActive.setPosition(ACTIVE_STABLE);
            }
        }
    }
}
