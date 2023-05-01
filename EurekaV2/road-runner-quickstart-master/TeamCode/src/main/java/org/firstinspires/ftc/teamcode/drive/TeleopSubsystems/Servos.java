package org.firstinspires.ftc.teamcode.drive.TeleopSubsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Servos {
//    public class Servos extends SubsystemBase {
    public static Servo servoGripper;
    public static Servo servoActive;
    public static Servo servoLeftF;
    public static Servo servoLeftB;
    public static Servo servoRightF;
    public static Servo servoRightB;
    public  static Servo servoRotate;

    public Servos(final HardwareMap hardwareMap, Telemetry telemetry) {
        servoGripper = hardwareMap.get(Servo.class, "servoGripper");
        servoActive = hardwareMap.get(Servo.class, "servoActive");
        servoLeftF = hardwareMap.get(Servo.class, "servoLeftF");
        servoLeftB = hardwareMap.get(Servo.class, "servoLeftB");
        servoRightF = hardwareMap.get(Servo.class, "servoRightF");
        servoRightB = hardwareMap.get(Servo.class, "servoRightB");
        servoRotate = hardwareMap.get(Servo.class, "servoRotate");
    }

    public static class Gripper {
        public static String gripperState = "OPEN";
        private static final double gripperOpenPosition = 0.32;
        private static final double gripperClosePosition = 0;

        public static void openGripper() {
            gripperState = "OPEN";
            servoGripper.setPosition(gripperOpenPosition);
        }

        public static void closeGripper() {
            gripperState = "CLOSED";
            servoGripper.setPosition(gripperClosePosition);
        }
    }

    public static class Rotate {
        public static String rotateState = "PICK";
        private static final double rotateDrop = 0.65;
        private static final double rotatePick = 0;

        public static void rotateDrop() {
            rotateState = "DROP";
            servoRotate.setPosition(rotateDrop);
        }
        public static void rotatePick() {
            rotateState = "PICK";
            servoRotate.setPosition(rotatePick);
        }
    }
    public static class Arm {

        public static String armState = "INIT";

        public static final double DropLeft = 0.12;
        public static final double DropRight = 1-DropLeft;
        public static final double Init = 0.5;
        public static final double UPLeft = 0.6;
        public static final double UPRight = 1-UPLeft;
        public static final double PickLeftOne = 0.78;
        public static final double PickRightOne = 1-PickLeftOne;
        public static final double PickLeftTwo = 0.82;
        public static final double PickRightTwo = 1-PickLeftTwo;
        public static final double PickLeftThree = 0.86;
        public static final double PickRightThree = 1-PickLeftThree;
        public static final double PickLeftFour = 0.9;
        public static final double PickRightFour = 1-PickLeftFour;
        public static final double PickLeftFive = 0.98; //0.95
        public static final double PickRightFive = 1-PickLeftFive; //0.05

        public static final double LegalLeft = 0.2;
        public static final double LegalRight = 1-LegalLeft;

        public static final double ActiveUp = 1;
        public static final double ActiveMid = 0.5;
        public static final double ActiveDown = 0;

        public static void goActiveUp() {
            armState = "ActiveUp";
            servoActive.setPosition(ActiveUp);
        }
        public static void goActiveDown() {
            armState = "ActiveDown";
            servoActive.setPosition(ActiveDown);
        }
        public static void goActiveMid() {
            armState = "ActiveMid";
            servoActive.setPosition(ActiveMid);
        }



        public static void goLegal() {
            armState = "Legal";
            servoLeftF.setPosition(LegalLeft);
            servoLeftB.setPosition(LegalLeft);
            servoRightF.setPosition(LegalRight);
            servoRightB.setPosition(LegalRight);
        }

        public static void goUP() {
            armState = "UP";
            servoLeftF.setPosition(UPLeft);
            servoLeftB.setPosition(UPLeft);
            servoRightF.setPosition(UPRight);
            servoRightB.setPosition(UPRight);
        }

        public static void goDrop() {
            armState = "DROP";
            servoLeftF.setPosition(DropLeft);
            servoLeftB.setPosition(DropLeft);
            servoRightF.setPosition(DropRight);
            servoRightB.setPosition(DropRight);
        }

        public static void goInit() {
            armState = "INIT";
            servoLeftF.setPosition(Init);
            servoLeftB.setPosition(Init);
            servoRightF.setPosition(Init);
            servoRightB.setPosition(Init);
        }


        public static void goPickCone1() {
            armState = "ONE";
            servoLeftF.setPosition(PickLeftOne);
            servoLeftB.setPosition(PickLeftOne);
            servoRightF.setPosition(PickRightOne);
            servoRightB.setPosition(PickRightOne);
        }

        public static void goPickCone2() {
            armState = "TWO";
            servoLeftF.setPosition(PickLeftTwo);
            servoLeftB.setPosition(PickLeftTwo);
            servoRightF.setPosition(PickRightTwo);
            servoRightB.setPosition(PickRightTwo);
        }

        public static void goPickCone3() {
            armState = "THREE";
            servoLeftF.setPosition(PickLeftThree);
            servoLeftB.setPosition(PickLeftThree);
            servoRightF.setPosition(PickRightThree);
            servoRightB.setPosition(PickRightThree);
        }

        public static void goPickCone4() {
            armState = "FOUR";
            servoLeftF.setPosition(PickLeftFour);
            servoLeftB.setPosition(PickLeftFour);
            servoRightF.setPosition(PickRightFour);
            servoRightB.setPosition(PickRightFour);
        }

        public static void goPickCone5() {
            armState = "FIVE";
            servoLeftF.setPosition(PickLeftFive);
            servoLeftB.setPosition(PickLeftFive);
            servoRightF.setPosition(PickRightFive);
            servoRightB.setPosition(PickRightFive);
        }
    }
}
