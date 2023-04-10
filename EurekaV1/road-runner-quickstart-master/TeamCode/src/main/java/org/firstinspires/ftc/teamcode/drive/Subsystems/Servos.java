package org.firstinspires.ftc.teamcode.drive.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Servos {
    public static Servo servoGripper;
    public static Servo servoLeft;
    public static Servo servoRight;
    public static Servo servoRotation;

    public Servos(HardwareMap hardwareMap, Telemetry telemetry) {
        servoGripper = hardwareMap.get(Servo.class, "servoGripper");
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");
        servoRotation = hardwareMap.get(Servo.class, "servoRotation");
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
            servoRotation.setPosition(rotateDrop);
        }
        public static void rotatePick() {
            rotateState = "PICK";
            servoRotation.setPosition(rotatePick);
        }
    }

    public static class Arm {

        public static String armState = "INIT";

        public static final double DropLeft = 0.12;
        public static final double DropRight = 0.88;
        public static final double Init = 0.5;
        public static final double UPLeft = 0.6;
        public static final double UPRight = 0.4;
        public static final double PickLeftOne = 0.78;
        public static final double PickRightOne = 0.22;
        public static final double PickLeftTwo = 0.82;
        public static final double PickRightTwo = 0.18;
        public static final double PickLeftThree = 0.86;
        public static final double PickRightThree = 0.14;
        public static final double PickLeftFour = 0.9;
        public static final double PickRightFour = 0.1;
        public static final double PickLeftFive = 0.98; //0.95
        public static final double PickRightFive = 0.02; //0.05


        public static final double LegalLeft = 0.2;
        public static final double LegalRight = 0.8;

        private static final double PickLeftOneMid = 0.76;
        private static final double PickRightOneMid = 0.24;

        public static void goLegal() {
            armState = "Legal";
            servoLeft.setPosition(LegalLeft);
            servoRight.setPosition(LegalRight);
        }

        public static void goUP() {
            armState = "UP";
            servoLeft.setPosition(UPLeft);
            servoRight.setPosition(UPRight);
        }

        public static void goDrop() {
            armState = "DROP";
            servoLeft.setPosition(DropLeft);
            servoRight.setPosition(DropRight);
        }

        public static void goInit() {
            armState = "INIT";
            servoLeft.setPosition(Init);
            servoRight.setPosition(Init);
        }

        public static void goPickCone1Mid() {
            armState = "ONEMID";
            servoLeft.setPosition(PickLeftOneMid);
            servoRight.setPosition(PickRightOneMid);
        }

        public static void goPickCone1() {
            armState = "ONE";
            servoLeft.setPosition(PickLeftOne);
            servoRight.setPosition(PickRightOne);
        }

        public static void goPickCone2() {
            armState = "TWO";
            servoLeft.setPosition(PickLeftTwo);
            servoRight.setPosition(PickRightTwo);
        }

        public static void goPickCone3() {
            armState = "THREE";
            servoLeft.setPosition(PickLeftThree);
            servoRight.setPosition(PickRightThree);
        }

        public static void goPickCone4() {
            armState = "FOUR";
            servoLeft.setPosition(PickLeftFour);
            servoRight.setPosition(PickRightFour);
        }

        public static void goPickCone5() {
            armState = "FIVE";
            servoLeft.setPosition(PickLeftFive);
            servoRight.setPosition(PickRightFive);
        }
    }
}
