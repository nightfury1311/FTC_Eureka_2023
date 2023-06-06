package org.firstinspires.ftc.teamcode.drive.TeleopSubsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//public class Servos {
    public class Servos extends SubsystemBase {
    public static Servo servoGripper;
    public static Servo servoLF;
    public static Servo servoLB;
    public static Servo servoRF;
    public static Servo servoRB;
    public static Servo servoRotate;
    public static Servo servoActive;
    public static Servo servoLock;

    public Servos(final HardwareMap hardwareMap, Telemetry telemetry) {
        servoGripper = hardwareMap.get(Servo.class, "servoGripper");
        servoLock = hardwareMap.get(Servo.class, "servoLock");
        servoActive = hardwareMap.get(Servo.class, "servoActive");
        servoLF = hardwareMap.get(Servo.class, "servoLF");
        servoLB = hardwareMap.get(Servo.class, "servoLB");
        servoRF = hardwareMap.get(Servo.class, "servoRF");
        servoRB = hardwareMap.get(Servo.class, "servoRB");
        servoRotate = hardwareMap.get(Servo.class, "servoRotate");
    }

    public static class Gripper {
        public static String gripperState = "OPEN";
        public static final double gripperOpenPosition = 0.5;
        public static final double gripperClosePosition = 0.7;

        public static final double Lock = 0;
        public static final double Unlock = 0.3;

        public static void openGripper() {
            gripperState = "OPEN";
            servoGripper.setPosition(gripperOpenPosition);
        }

        public static void closeGripper() {
            gripperState = "CLOSE";
            servoGripper.setPosition(gripperClosePosition);
        }

        public static void Lock() {
            gripperState = "LOCK";
            servoLock.setPosition(Lock);
        }
        public static void Unlock() {
            gripperState = "UNLOCK";
            servoLock.setPosition(Unlock);
        }
    }

    public static class Rotate {
        public static String rotateState = "PICK";
        public static final double rotateDrop = 0.82;
        public static final double rotatePick = 0.17;

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

        public static final double DropLeft = 0.27;
        public static final double DropRight = 1-DropLeft;
        public static final double Init = 0.5;
        public static final double PickLeftOne = 0.83;
        public static final double PickRightOne = 1-PickLeftOne;
        public static final double PickLeftTwo = 0.88;
        public static final double PickRightTwo = 1-PickLeftTwo;
        public static final double PickLeftThree = 0.91;
        public static final double PickRightThree = 1-PickLeftThree;
        public static final double PickLeftFour = 0.96;
        public static final double PickRightFour = 1-PickLeftFour;

        public static final double PickLeft = 0.98;
        public static final double PickRight = 1-PickLeft;

        public static final double LegalLeft = 0.4;
        public static final double LegalRight = 1-LegalLeft;

        public static final double ActivePick = 0.88;
        public static final double ActivePick4 = 0.86;
        public static final double ActivePick3 = 0.85;
        public static final double ActivePick2 = 0.82;
        public static final double ActivePick1 = 0.76;
        public static final double ActiveStable = 0.5;
        public static final double ActiveDrop = 0.35;

        public static final double ActiveLow = 0.69;
        public static void goActiveLow() {
            armState = "ActiveLow";
            servoActive.setPosition(ActiveLow);
        }

        public static void goActivePick() {
            armState = "ActivePick";
            servoActive.setPosition(ActivePick);
        }
        public static void goActiveStable() {
            armState = "ActiveStable";
            servoActive.setPosition(ActiveStable);
        }
        public static void goActiveDrop() {
            armState = "ActiveDrop";
            servoActive.setPosition(ActiveDrop);
        }
        public static void goActivePick1() {
            armState = "ActivePick1";
            servoActive.setPosition(ActivePick1);
        }
        public static void goActivePick2() {
            armState = "ActivePick2";
            servoActive.setPosition(ActivePick2);
        }
        public static void goActivePick3() {
            armState = "ActivePick3";
            servoActive.setPosition(ActivePick3);
        }
        public static void goActivePick4() {
            armState = "ActivePick4";
            servoActive.setPosition(ActivePick4);
        }



        public static void goLegal() {
            armState = "Legal";
            servoLF.setPosition(LegalLeft);
            servoLB.setPosition(LegalLeft);
            servoRF.setPosition(LegalRight);
            servoRB.setPosition(LegalRight);
        }

        public static void goPick() {
            armState = "PICK";
            servoLF.setPosition(PickLeft);
            servoLB.setPosition(PickLeft);
            servoRF.setPosition(PickRight);
            servoRB.setPosition(PickRight);
        }
        public static void goDrop() {
            armState = "DROP";
            servoLF.setPosition(DropLeft);
            servoLB.setPosition(DropLeft);
            servoRF.setPosition(DropRight);
            servoRB.setPosition(DropRight);
        }

        public static void goInit() {
            armState = "INIT";
            servoLF.setPosition(Init);
            servoLB.setPosition(Init);
            servoRF.setPosition(Init);
            servoRB.setPosition(Init);
        }


        public static void goPickCone1() {
            armState = "ONE";
            servoLF.setPosition(PickLeftOne);
            servoLB.setPosition(PickLeftOne);
            servoRF.setPosition(PickRightOne);
            servoRB.setPosition(PickRightOne);
        }

        public static void goPickCone2() {
            armState = "TWO";
            servoLF.setPosition(PickLeftTwo);
            servoLB.setPosition(PickLeftTwo);
            servoRF.setPosition(PickRightTwo);
            servoRB.setPosition(PickRightTwo);
        }

        public static void goPickCone3() {
            armState = "THREE";
            servoLF.setPosition(PickLeftThree);
            servoLB.setPosition(PickLeftThree);
            servoRF.setPosition(PickRightThree);
            servoRB.setPosition(PickRightThree);
        }

        public static void goPickCone4() {
            armState = "FOUR";
            servoLF.setPosition(PickLeftFour);
            servoLB.setPosition(PickLeftFour);
            servoRF.setPosition(PickRightFour);
            servoRB.setPosition(PickRightFour);
        }

    }
}
