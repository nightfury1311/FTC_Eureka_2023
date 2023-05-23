package org.firstinspires.ftc.teamcode.drive.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.drive.CommandBase.ElevatorHigh;

import org.firstinspires.ftc.teamcode.drive.CommandBase.ElevatorHome;
import org.firstinspires.ftc.teamcode.drive.CommandBase.ElevatorMid;
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Elevator;

import java.util.function.BooleanSupplier;
import  com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.motors.RevRoboticsUltraPlanetaryHdHexMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "EurekaCommandBase")
public class EurekaCommandBase extends CommandOpMode {

    Elevator elevator;
    GamepadEx driver;
    Button  liftHighButton, liftHomeButton, liftMidButton;
    @Override
    public void initialize() {

        driver = new GamepadEx(gamepad1);

        liftHighButton = new GamepadButton(driver, GamepadKeys.Button.DPAD_UP);
        liftMidButton = new GamepadButton(driver, GamepadKeys.Button.Y);
        liftHomeButton = new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN);

        this.elevator = new Elevator(hardwareMap);

        liftHighButton.whenPressed(new ParallelCommandGroup(new ElevatorHigh(elevator)));
        liftMidButton.whenPressed(new ParallelCommandGroup(new ElevatorMid(elevator)));
        liftHomeButton.whenPressed(new ParallelCommandGroup(new ElevatorHome(elevator)));


            }
        }



