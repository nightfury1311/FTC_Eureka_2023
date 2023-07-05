package org.firstinspires.ftc.teamcode.drive.TeleopSubsystems;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//public class Slide {
public class SliderEx extends SubsystemBase {
    public DcMotorEx SlideLeft, SlideRight;

    public ElapsedTime timer;
    public double targetX;

    MotionProfile motionProfilex;

    public int HOME = 0, MICRO=1, MIN = 2, CONE1 = 3, CONE2= 4, CONE3=5, CONE4=6, CONE5=7, MAX = 8 ,UNSAFE=9, TELETEST=10, TELEMAX=11, MIDMICRO=12, MID=13;
    //    public static int[] POSITIONS = {0,610, 630, 1330, 1330, 1280, 1280, 1280, 1320 ,530};
    public static int[] POSITIONS = {0,390, 355, 670, 670, 670, 670, 670, 670 ,200, 275, 690, 300, 390};

    public SliderEx(HardwareMap hardwareMap, Telemetry telemetry) {
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

    public void set(double sliderPower) {
        SlideLeft.setPower(sliderPower);
        SlideRight.setPower(sliderPower);

        sliderPower = Range.clip(sliderPower, -1, 1);
    }
    public void extendToSlow(int position){

        SlideLeft.setTargetPosition(position);
        SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideLeft.setPower(0.7);

        SlideRight.setTargetPosition(position);
        SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideRight.setPower(0.7);
    }
    public double getPosition(){
        return SlideRight.getCurrentPosition();
    }

    public double getPositionEx(){
        return SlideRight.getCurrentPosition();
    }

    public void reset() {
        SlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //Motion Profiling
    public void goTo(double x, double maxVel, double maxAccel) {
        targetX = x;
        double currentx = SlideRight.getCurrentPosition();
        motionProfilex = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentx, 0, 0),
                new MotionState(x, 0, 0),
                maxVel,
                maxAccel
        );
        timer.reset();
    }

    public double update() {
        if (motionProfilex != null) {
            MotionState xState = motionProfilex.get(timer.seconds());

            return xState.getX();
        }
        return 0;
    }
    public double getCurrent(){
        return SlideLeft.getCurrent(CurrentUnit.MILLIAMPS);
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
    public void extendToTELETEST(){extendTo(POSITIONS[TELETEST]);}
    public void extendToTELEMAX(){extendTo(POSITIONS[TELEMAX]);}


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
    public void extendToUNSAFESlow(){extendTo(POSITIONS[UNSAFE]);}
    public void extendToTELETESTSlow(){extendTo(POSITIONS[TELETEST]);}
    public void extendToTELEMAXSlow(){extendTo(POSITIONS[TELEMAX]);}

}

