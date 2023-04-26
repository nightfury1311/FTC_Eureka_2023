package org.firstinspires.ftc.teamcode.drive.MotionProfile;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@Config
public class elevator_sub extends SubsystemBase {

    MotorEx leftMotor, rightMotor;
    HardwareMap hardwareMap;;
    Telemetry telemetry;

    public static double targetCounts = 0;
    private final double ticks_in_degree = (5 * 28)/360;

    private PIDController leftController;
    private PIDController rightController;
    public static double Kp = 0.03;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 0.1;

    public final int HOME = 0, LOW_POLE = 1, MID_POLE = 2, HIGH_POLE = 3;
    public final int[] POSITIONS = {0, 300, 600, 1100};  //Junction Height for dropping cones normally

    private double leftPower = 0;
    private double rightPower = 0;


    public elevator_sub(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        leftController = new PIDController(Kp, Ki, Kd);
        rightController = new PIDController(Kp, Ki, Kd);
        leftMotor = hardwareMap.get(MotorEx.class, "ElevateLeft");
        rightMotor = hardwareMap.get(MotorEx.class, "ElevateRight");

        leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        leftController.setTolerance(1);

        leftMotor.setRunMode(Motor.RunMode.RawPower);
        rightMotor.setRunMode(Motor.RunMode.RawPower);
        leftController.setPID(Kp, Ki, Kd);
    }


    @Override
    public void periodic() {
        super.periodic();
        leftController.setPID(Kp, Ki, Kd);
        rightController.setPID(Kp, Ki, Kd);
        double rightCurrentCounts = rightMotor.getCurrentPosition();
        double leftCurrentCounts = leftMotor.getCurrentPosition();
        double leftPid = leftController.calculate(leftCurrentCounts, targetCounts);
        double rightPid = rightController.calculate(rightCurrentCounts, targetCounts);

        double ff = Math.cos(Math.toRadians(targetCounts/ticks_in_degree)) * Kf;
//        telemetry.addData("Position: ", currentCounts);
        leftPower = leftPid + ff;
        rightPower = rightPid + ff;

        write();

    }

    public void write(){
        this.setPower(this.leftPower, this.rightPower);
    }

    public void setPower(double power1, double power2){
        leftMotor.set(power1);
        rightMotor.set(power2);
    }

    public void extendTo(double pos){
        targetCounts = pos;
    }

    public void extendToHighPole(){
        extendTo(POSITIONS[HIGH_POLE]);
    }
    public void extendToHome(){
        extendTo(POSITIONS[HOME]);
    }

    public double[] getPosition(){
        return new double[] {leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition()};
    }

    public void reset(){
        leftMotor.resetEncoder();
        rightMotor.resetEncoder();
    }



}