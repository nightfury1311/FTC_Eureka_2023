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

    public int HOME = 0, MIN = 1, CONE1 = 2, CONE2= 3, CONE3=4, CONE4=5, CONE5=6, MAX = 7, MAXMID = 8, ULTRAMID=9, ULTRAPROMAX=10, PRE=11;
    public static int[] POSITIONS = {0, 310, 1360, 1320, 1300, 1270, 1300, 1420, 1450, 1550, 1700, 320 };
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
    public void extendToMAX(){
        extendTo(POSITIONS[MAX]);
    }
    public void extendToMAXMID(){
        extendTo(POSITIONS[MAXMID]);
    }
    public void extendToULTRAMID(){
        extendTo(POSITIONS[ULTRAMID]);
    }
    public void extendToULTRAPROMAX(){
        extendTo(POSITIONS[ULTRAPROMAX]);
    }
    public void extendToPRE(){
        extendTo(POSITIONS[PRE]);
    }

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
    public void extendToMAXMIDSlow(){
        extendToSlow(POSITIONS[MAXMID]);
    }
    public void extendToULTRAMIDSlow(){
        extendToSlow(POSITIONS[ULTRAMID]);
    }
    public void extendToULTRAPROMAXSlow(){
        extendToSlow(POSITIONS[ULTRAPROMAX]);
    }
    public void extendToPRESlow(){
        extendToSlow(POSITIONS[PRE]);
    }

}

