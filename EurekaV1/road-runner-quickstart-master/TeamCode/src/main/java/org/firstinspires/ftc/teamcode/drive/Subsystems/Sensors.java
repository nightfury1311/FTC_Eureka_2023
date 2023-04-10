package org.firstinspires.ftc.teamcode.drive.Subsystems;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Sensors {
    private static AnalogInput ultrasoundSensor = null;
    private static RevColorSensorV3 gripperSensor = null;
    private static Rev2mDistanceSensor poleRight = null;
    private static Rev2mDistanceSensor poleMid = null;
    private static Rev2mDistanceSensor poleLeft = null;

    private static Telemetry localTelemetry;
    public Sensors(HardwareMap hardwareMap, Telemetry telemetry){
        localTelemetry = telemetry;
//        ultrasoundSensor = hardwareMap.get(AnalogInput.class, "ultrasound1");
//        gripperSensor = hardwareMap.get(RevColorSensorV3.class, "gripperSensor");
        poleRight = hardwareMap.get(Rev2mDistanceSensor.class, "poleRight");
        poleMid = hardwareMap.get(Rev2mDistanceSensor.class, "poleMid");
        poleLeft = hardwareMap.get(Rev2mDistanceSensor.class, "poleLeft");

    }

    public static class PoleRight{
        public static double getDistanceCM(){
            return poleRight.getDistance(DistanceUnit.CM);
        }
    }
    public static class PoleMid{
        public static double getDistanceCM(){
            return poleMid.getDistance(DistanceUnit.CM);
        }
    }
    public static class PoleLeft{
        public static double getDistanceCM(){
            return poleLeft.getDistance(DistanceUnit.CM);
        }
    }


//    public static class WallSensor{
//
//        public static double lastDistance = 0;
//        public static double getDistanceCM(){
//            lastDistance = ultrasoundSensor.getVoltage() * 520.00/3.3;
//            return lastDistance;
//        }
//
//        public static void printDistance(){
//            localTelemetry.addData("Wall Distance: ", getDistanceCM());
//        }
//    }

//    public static class GripperSensor{
//        public static double getDistanceMM(){
//            double dist = gripperSensor.getDistance(DistanceUnit.MM);
//            return dist;
//        }
//
//        public static double getDistanceINCH(){
//            double dist = gripperSensor.getDistance(DistanceUnit.INCH);
//            return dist;
//        }
//
//        public static boolean checkRed(){
//            if(gripperSensor.red() > 200){
//                return true;
//            }
//            else{
//                return false;
//            }
//        }
//
//        public static boolean checkBlue(){
//            if(gripperSensor.blue() > 200){
//                return true;
//            }
//            else{
//                return false;
//            }
//        }
//
//    }
}
