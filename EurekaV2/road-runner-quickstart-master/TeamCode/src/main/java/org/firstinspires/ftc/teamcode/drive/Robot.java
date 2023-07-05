package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Elevator;
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.ElevatorEx;
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.SliderEx;

@Config
public class Robot extends SampleMecanumDrive {

    public static double Kp_elevator = 0.006;
    public static double Ki_elevator = 0;
    public static double Kd_elevator = 0;
    public static double Kf_elevator = -0.14;

    public static double Kp_slider = 0.008;
    public static double Ki_slider = 0;
    public static double Kd_slider = 0;
    public static double Kf_slider = -0.2;

    private Telemetry localTelemetry;
    ElevatorEx robotElevator ;
    SliderEx robotSlider ;

    private PIDController elevatorExController;
    private PIDController sliderExController;

    public static double targetElevator = 0;
    public static double targetSlider = 0;


    public Robot(HardwareMap hardwareMap, Telemetry localTelemetry, ElevatorEx elevatorEx, SliderEx sliderEx) {
        super(hardwareMap, localTelemetry);
        robotElevator = elevatorEx;
        robotSlider = sliderEx;

        elevatorExController = new PIDController(Kp_elevator, Ki_elevator, Kd_elevator);
        sliderExController = new PIDController(Kp_slider, Ki_slider, Kd_slider);

        this.localTelemetry = localTelemetry;
    }

    @Override
    public void update() {
        super.update();
        /// SLIDER
        elevatorExController.setPID(Kp_elevator, Ki_elevator, Kd_elevator);
        double currentElevatorPosition = robotElevator.getPositionEx();
        double ElevatorPID = elevatorExController.calculate(currentElevatorPosition, robotElevator.update());
        double ff_Elevator = 1*Kf_elevator;
        double elevatorPower = ff_Elevator + ElevatorPID;

        elevatorPower = Range.clip(elevatorPower,-1,1);

        robotElevator.set(elevatorPower);

        sliderExController.setPID(Kp_slider, Ki_slider, Kd_slider);
        double currentSliderPosition = robotSlider.getPositionEx();
        double SliderPID = sliderExController.calculate(currentSliderPosition, robotSlider.update());
        double ff_Slider = 1*Kf_slider;
        double sliderPower = ff_Slider + SliderPID;

        sliderPower = Range.clip(sliderPower,-1,1);

        robotSlider.set(sliderPower);

        this.localTelemetry.addData("Elevator Power", elevatorPower);
        this.localTelemetry.addData("Elevator Target", targetElevator);
        this.localTelemetry.addData("Elevator Current: ", currentElevatorPosition);
        this.localTelemetry.addData("elevator",robotElevator.getPosition());
        this.localTelemetry.addData("Elevator rightelevator",robotElevator.getPosition());

    }
}