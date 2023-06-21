package org.firstinspires.ftc.teamcode.drive.TeleopSubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

//public class Elevator{
    public class Elevator extends SubsystemBase {
        public DcMotorEx ElevateLeft, ElevateRight;

        public final int HOME = 0, LOW_POLE = 1, MID_POLE = 2, HIGH_POLE = 3;
        //    public final int[] POSITIONS = {0, 600, 1130, 1765, 20};     //OLD VALUES at 9:1 elevator ratio
//        public final int[] POSITIONS = {0, 330, 700, 1110, 30};       //NEW VALUES at 5:1 elevator ratio
//        public final int[] POSITIONS = {0, 228, 484, 750, 30};       //NEW VALUES at 4:1 elevator ratio
            public final int[] POSITIONS = {0, 183, 387, 600, 10};       //NEW VALUES at 3:1 elevator ratio 1.24

    public Elevator(final HardwareMap hardwareMap, Telemetry telemetry) {
        ElevateLeft = hardwareMap.get(DcMotorEx.class, "ElevateLeft");
        ElevateRight = hardwareMap.get(DcMotorEx.class, "ElevateRight");

      ElevateLeft.setDirection(DcMotorEx.Direction.REVERSE);
        ElevateRight.setDirection(DcMotorEx.Direction.REVERSE);
    }
        public void extendTo(int position){
            ElevateLeft.setTargetPosition(position);
            ElevateLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            ElevateLeft.setPower(1);

            ElevateRight.setTargetPosition(position);
            ElevateRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            ElevateRight.setPower(1);
        }

    public void extendToSlow(int position){
        ElevateLeft.setTargetPosition(position);
        ElevateLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        ElevateLeft.setPower(0.5);

        ElevateRight.setTargetPosition(position);
        ElevateRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        ElevateRight.setPower(0.5);
    }
    public double[] getPosition(){
        return new double[]{ElevateLeft.getCurrentPosition(), ElevateRight.getCurrentPosition()};
    }
    public void reset() {
        ElevateLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElevateRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public double[] getCurrent(){
        return new double[]{
                ElevateLeft.getCurrent(CurrentUnit.MILLIAMPS), ElevateRight.getCurrent(CurrentUnit.MILLIAMPS)};
    }
        public void extendToHighPole(){
            extendTo(POSITIONS[HIGH_POLE]);
        }
        public void extendToMidPole(){
            extendTo(POSITIONS[MID_POLE]);
        }
        public void extendToLowPole(){
            extendTo(POSITIONS[LOW_POLE]);
        }
        public void extendToHome(){
            extendTo(POSITIONS[HOME]);
        }

    public void extendToHP(){
        extendToSlow(POSITIONS[HIGH_POLE]);
    }
    public void extendToMP(){
        extendToSlow(POSITIONS[MID_POLE]);
    }
    public void extendToLP(){
        extendToSlow(POSITIONS[LOW_POLE]);
    }
    public void extendToH(){
        extendToSlow(POSITIONS[HOME]);
    }

}
