package org.firstinspires.ftc.teamcode.drive.TeleopSubsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.geometry.AsymmetricMotionProfile;


@Config
public class SlidersEx {
    private AsymmetricMotionProfile liftProfile;
    public static double POWER = -1;
    public static boolean powerflag = false;
    public double maxVel = 100000;
    public double maxAccel = 4000;
    DcMotorEx leftSlider, rightSlider;



    private Telemetry localTelemetry;
    public static String currentSliderState = "HOME";
    public static String prevSliderState = "FULL";

    public static String sliderState = "CLOSE";


    public int encoderValue;


    //motion profile
    ElapsedTime timer;
    public double targetX;
    MotionProfile motionProfilex;

    //Making slider Position for different cones *************************************
    public int HOME = 0, CONE1 = 1, CONE2= 2, CONE3=3, CONE4=4, CONE5=5, MAX = 6 ;
    public static int[] POSI = {0,300,500,0,0,0,1170};//*****************************************


    public SlidersEx(HardwareMap hardwareMap, Telemetry localTelemetry) {

        this.localTelemetry = telemetry;

        leftSlider = hardwareMap.get(DcMotorEx.class, "leftSlider");
        rightSlider = hardwareMap.get(DcMotorEx.class, "rightSlider");

        leftSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//        leftSlider.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightSlider.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlider.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlider.setDirection(DcMotorSimple.Direction.FORWARD);
        timer = new ElapsedTime();



        timer= new ElapsedTime();

    }

    public void set(double power) {

        power = Range.clip(power, -1, 1);

        leftSlider.setPower(power);
        rightSlider.setPower(power);

//        this.localTelemetry.addData("slider Pow",leftSlider.getPower());
    }


    public void setSlider(int pos) {

        int encoderValue = leftSlider.getCurrentPosition();

        leftSlider.setTargetPosition(pos);
        leftSlider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftSlider.setPower(POWER);
//        rightSlider.setPower(leftSlider.getPower());   //For Making it follower.
        rightSlider.setTargetPosition(pos);
        rightSlider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightSlider.setPower(POWER);

    }


    public double getPosition() {
        return leftSlider.getCurrentPosition();
    }

    public double rightSliderCurrentPos(){
        return rightSlider.getCurrentPosition();
    }



    public double getCurrent() {
        return leftSlider.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public void reset() {
//        leftSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    //Motion Profiling
    public void goTo(double x, double maxVel, double maxAccel) {
        targetX = x;
        double currentx = leftSlider.getCurrentPosition();
        motionProfilex = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentx, 0, 0),
                new MotionState(x, 0, 0),
                maxVel,
                maxAccel
        );
        timer.reset();

    }



    public void goTo(double x) {

        if(x > 0){
            sliderState = "OPEN";
        }
        else if (x == 0){
            sliderState = "CLOSE";
        }

        targetX = x;
        double currentSliderPos = leftSlider.getCurrentPosition();

        motionProfilex = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentSliderPos, 0, 0),
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





}