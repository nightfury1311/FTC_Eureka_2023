package org.firstinspires.ftc.teamcode.drive.Teleop;

import static android.os.SystemClock.sleep;

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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Servos;

//@Disabled
@Config
@TeleOp
public class SLPID extends OpMode {
    private PIDController controller;

    Servos servos = null;


    public static double p = 0.008, i = 0, d = 0;
    public static double f = 0, ff = 0.14;
//    public static double ff = 0.12;

    public static int target = 0;
    private final double ticks_in_degree = (3 * 28)/360;

    private DcMotorEx SlideLeft, SlideRight;

    @Override
    public void init(){
        controller= new PIDController(p,i,d);

        telemetry= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        servos = new Servos(hardwareMap, telemetry);

        SlideLeft = hardwareMap.get(DcMotorEx.class, "SlideLeft");
        SlideRight = hardwareMap.get(DcMotorEx.class, "SlideRight");
        SlideRight.setDirection(DcMotorSimple.Direction.REVERSE);

        target = 0;
        Servos.Gripper.openGripper();
        Servos.Arm.goActiveStable();
        Servos.Arm.goInit();
        Servos.Rotate.rotatePick();
    }
    @Override
    public void loop(){
        controller.setPID(p, i, d);

        int SlideFinalPos = SlideRight.getCurrentPosition();
        double power = Range.clip(((controller.calculate(SlideFinalPos, target)+ ff)) , -0.9, 0.9);
        SlideLeft.setPower(power);
        SlideRight.setPower(power);
        double error = target-SlideFinalPos;


        if(gamepad1.left_bumper) {
            target=275;
        }
        else if (gamepad1.back) {
            target=0;
        }
        else if (gamepad1.dpad_left){
            Servos.Gripper.openGripper();
            Servos.Arm.goActivePick();
            Servos.Rotate.rotatePick();
            sleep(200);
            Servos.Arm.goPickTele();
        }
        else if (gamepad1.dpad_right) {
            Servos.Gripper.closeGripper();
            sleep(200);
            Servos.Arm.goInit();
            Servos.Arm.goActiveLow();
        }



        telemetry.addData("FinalPos", SlideFinalPos);
        telemetry.addData("target", target);
        telemetry.addData("Power: ", power*200);
        telemetry.addData("Current SlideRight", SlideRight.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Current SlideLeft", SlideLeft.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("SlideRight", SlideRight.getCurrentPosition());
        telemetry.addData("SlideLeft", SlideLeft.getCurrentPosition());
        telemetry.addData("error:", error);
        telemetry.update();
    }

}
