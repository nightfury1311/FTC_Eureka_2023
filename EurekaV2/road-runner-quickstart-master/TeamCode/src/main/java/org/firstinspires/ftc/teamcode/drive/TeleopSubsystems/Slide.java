package org.firstinspires.ftc.teamcode.drive.TeleopSubsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
public class Slide {
    public DcMotorEx SlideLeft, SlideRight;

    public int HOME = 0, MICRO=1, MIN = 2, CONE1 = 3, CONE2= 4, CONE3=5, CONE4=6, CONE5=7, MAX = 8 ,UNSAFE=9;
//    public static int[] POSITIONS = {0,610, 630, 1330, 1330, 1280, 1280, 1280, 1320 ,530};
public static int[] POSITIONS = {0,335, 348, 732, 732, 707, 707, 707, 729 ,292};

    public Slide(HardwareMap hardwareMap, Telemetry telemetry) {
        SlideLeft = hardwareMap.get(DcMotorEx.class, "SlideLeft");
        SlideRight = hardwareMap.get(DcMotorEx.class, "SlideRight");
        SlideRight.setDirection(DcMotorEx.Direction.REVERSE);
//        SlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void extendTo(int position){

        SlideLeft.setTargetPosition(position);
        SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideLeft.setPower(1);

        SlideRight.setTargetPosition(position);
        SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideRight.setPower(1);
    }
    public void extendToSlow(int position){

        SlideLeft.setTargetPosition(position);
        SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideLeft.setPower(0.6);

        SlideRight.setTargetPosition(position);
        SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideRight.setPower(0.6);
    }
    public double[] getPosition(){
        return new double[]{SlideLeft.getCurrentPosition(), SlideRight.getCurrentPosition()};
    }
    public void reset() {
        SlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public double[] getCurrent(){
        return new double[]{
                SlideLeft.getCurrent(CurrentUnit.MILLIAMPS), SlideRight.getCurrent(CurrentUnit.MILLIAMPS)};
    }
    public void extendToHome(){
        extendTo(POSITIONS[HOME]);
    }
    public void extendToCONE1(){extendTo(POSITIONS[CONE1]);}
    public void extendToCONE2(){extendTo(POSITIONS[CONE2]);}
    public void extendToCONE3(){extendTo(POSITIONS[CONE3]);}
    public void extendToCONE4(){extendTo(POSITIONS[CONE4]);}
    public void extendToCONE5(){extendTo(POSITIONS[CONE5]);}
    public void extendToMIN(){
        extendTo(POSITIONS[MIN]);
    }
    public void extendToMICRO(){
        extendTo(POSITIONS[MICRO]);
    }
    public void extendToMAX(){
        extendTo(POSITIONS[MAX]);
    }
    public void extendToUNSAFE(){extendTo(POSITIONS[UNSAFE]);}

    public void extendToHomeSlow(){
        extendToSlow(POSITIONS[HOME]);
    }
    public void extendToCONE1Slow(){extendToSlow(POSITIONS[CONE1]);}
    public void extendToCONE2Slow(){extendToSlow(POSITIONS[CONE2]);}
    public void extendToCONE3Slow(){extendToSlow(POSITIONS[CONE3]);}
    public void extendToCONE4Slow(){extendToSlow(POSITIONS[CONE4]);}
    public void extendToCONE5Slow(){extendToSlow(POSITIONS[CONE5]);}
    public void extendToMINSlow(){
        extendToSlow(POSITIONS[MIN]);
    }
    public void extendToMAXSlow(){
        extendToSlow(POSITIONS[MAX]);
    }

}

