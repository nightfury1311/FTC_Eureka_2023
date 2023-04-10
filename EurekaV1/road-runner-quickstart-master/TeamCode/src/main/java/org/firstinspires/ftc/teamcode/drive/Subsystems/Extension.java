package org.firstinspires.ftc.teamcode.drive.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Extension {

    DcMotorEx ElevateLeft, ElevateRight;


    public final int HOME = 0, LOW_POLE = 1, MID_POLE = 2, HIGH_POLE = 3, FIRST = 4;
//    public final int[] POSITIONS = {0, 600, 1130, 1765, 20};     //OLD VALUES at 9:1 elevator ratio
    public final int[] POSITIONS = {0, 330, 700, 1110, 30};       //NEW VALUES at 5:1 elevator ratio
    public Extension(HardwareMap hardwareMap, Telemetry telemetry) {
        ElevateLeft = hardwareMap.get(DcMotorEx.class, "ElevateLeft");
        ElevateRight = hardwareMap.get(DcMotorEx.class, "ElevateRight");
    }
    public void extendTo(int position){

        ElevateLeft.setTargetPosition(position);
        ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ElevateLeft.setPower(0.85);

        ElevateRight.setTargetPosition(position);
        ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ElevateRight.setPower(0.85);
    }
    public void extendToFast(int position){

        ElevateLeft.setTargetPosition(position);
        ElevateLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ElevateLeft.setPower(0.9);

        ElevateRight.setTargetPosition(position);
        ElevateRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ElevateRight.setPower(0.9);
    }
    public double[] getPosition(){
        return new double[]{ElevateLeft.getCurrentPosition(), ElevateRight.getCurrentPosition()};
    }
    public void reset() {
        ElevateLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElevateRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public double[] getCurrent(){
        return new double[]{ElevateLeft.getCurrent(CurrentUnit.MILLIAMPS), ElevateRight.getCurrent(CurrentUnit.MILLIAMPS)};
    }
    public void extendToHighPole(){
        extendTo(POSITIONS[HIGH_POLE]);
    }
    public void extendToMidPole(){
        extendTo(POSITIONS[MID_POLE]);
    }
    public void extendToHome(){
        extendTo(POSITIONS[HOME]);
    }
    public void extendToFIRST(){
        extendTo(POSITIONS[FIRST]);
    }
    public void extendToFIRSTFast(){
        extendToFast(POSITIONS[FIRST]);
    }
    public void extendToHomeFast(){
        extendToFast(POSITIONS[HOME]);
    }
}

