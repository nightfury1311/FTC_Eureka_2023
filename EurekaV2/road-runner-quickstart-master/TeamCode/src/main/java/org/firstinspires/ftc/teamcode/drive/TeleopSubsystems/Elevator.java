package org.firstinspires.ftc.teamcode.drive.TeleopSubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

    public class Elevator extends SubsystemBase {

        DcMotorEx ElevateLeft, ElevateRight;

        public final int HOME = 0, LOW_POLE = 1, MID_POLE = 2, HIGH_POLE = 3;
        //    public final int[] POSITIONS = {0, 600, 1130, 1765, 20};     //OLD VALUES at 9:1 elevator ratio
//        public final int[] POSITIONS = {0, 330, 700, 1110, 30};       //NEW VALUES at 5:1 elevator ratio
            public final int[] POSITIONS = {0, 183, 387, 620, 10};       //NEW VALUES at 3:1 elevator ratio

    public Elevator(final HardwareMap hMap) {
        ElevateLeft = hMap.get(DcMotorEx.class, "ElevateLeft");
        ElevateRight = hMap.get(DcMotorEx.class, "ElevateRight");
    }
        public void extendTo(int position){
            ElevateLeft.setTargetPosition(position);
            ElevateLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            ElevateLeft.setPower(0.9);

            ElevateRight.setTargetPosition(position);
            ElevateRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            ElevateRight.setPower(0.9);
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

}
