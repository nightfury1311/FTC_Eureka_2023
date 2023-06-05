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
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.SlidersEx;

@Config
public class Robot extends SampleMecanumDrive {

    public static double Kp_slider = 0.01;
    public static double Ki_slider = 0;
    public static double Kd_slider = 0;
    public static double Kf_slider = 0;

    private Telemetry localTelemetry;
    SlidersEx robotSlider ;

    private PIDController sliderController;

    public static double targetSlider = 0;


    public Robot(HardwareMap hardwareMap, Telemetry localTelemetry, SlidersEx slider) {
        super(hardwareMap, localTelemetry);
        robotSlider = slider;

        sliderController = new PIDController(Kp_slider, Ki_slider, Kd_slider);

        this.localTelemetry = localTelemetry;
    }

    @Override
    public void update() {
        super.update();
        /// SLIDER
        sliderController.setPID(Kp_slider, Ki_slider, Kd_slider);
        double currentSliderPosition = robotSlider.getPosition();
        double SliderPID = sliderController.calculate(currentSliderPosition, robotSlider.update());
        double ff_Slider = 1*Kf_slider;
        double sliderPower = ff_Slider + SliderPID;

        sliderPower = Range.clip(sliderPower,-1,1);

        robotSlider.set(sliderPower);

        this.localTelemetry.addData("Slider Power", sliderPower);
        this.localTelemetry.addData("Slider Target", targetSlider);
        this.localTelemetry.addData("Slider Current: ", currentSliderPosition);
        this.localTelemetry.addData("slider",robotSlider.getPosition());
        this.localTelemetry.addData("Slider rightslider: ",robotSlider.rightSliderCurrentPos());
        this.localTelemetry.addData("Slider leftslider",robotSlider.getPosition());

    }
}