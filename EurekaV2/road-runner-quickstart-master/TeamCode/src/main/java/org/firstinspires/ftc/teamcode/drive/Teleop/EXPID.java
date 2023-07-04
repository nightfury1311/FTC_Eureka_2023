package org.firstinspires.ftc.teamcode.drive.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.controller.PIDController;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Disabled
@Config
@TeleOp
public class EXPID extends OpMode {
    private PIDController controller;

    public static double p = 0.004, i = 0, d = 0.04;
    public static double f = 0, ff = 0.14;
//    public static double ff = 0.12;

    public static int target = 0;
    private final double ticks_in_degree = (3 * 28)/360;

    private DcMotorEx ElevateLeft, ElevateRight;



    @Override
    public void init(){
        controller= new PIDController(p,i,d);

        telemetry= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ElevateLeft = hardwareMap.get(DcMotorEx.class, "ElevateLeft");
        ElevateRight = hardwareMap.get(DcMotorEx.class, "ElevateRight");
        ElevateLeft.setDirection(DcMotorEx.Direction.REVERSE);
        ElevateRight.setDirection(DcMotorEx.Direction.REVERSE);
    }
    @Override
    public void loop(){
        controller.setPID(p, i, d);

        int ElevateFinalPos = ElevateRight.getCurrentPosition();
        double pid = controller.calculate(ElevateFinalPos, target);
//        double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;

//        double power = pid;

        double power = Range.clip(((controller.calculate(ElevateFinalPos, target)+ ff)) , -1, 0.9);
        ElevateLeft.setPower(power);
        ElevateRight.setPower(power);
        double error = target-ElevateFinalPos;


//        if(gamepad1.dpad_up)
//        {
//            target=770;
//        } else if (gamepad1.dpad_down) {
//            target=0;
//        }
//        else if (gamepad1.y) {
//            target=484;
//        }
//        else if (gamepad1.a) {
//            target=228;
//        }

//        telemetry.addData("Left", ElevateLeftPos);
//        telemetry.addData("Right", ElevateRightPos);
        telemetry.addData("FinalPos", ElevateFinalPos);
        telemetry.addData("target", target);
        telemetry.addData("Power: ", power*200);
        telemetry.addData("Current ElevateRight", ElevateRight.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Current ElevateLeft", ElevateLeft.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("error:", error);
        telemetry.update();
    }

}
