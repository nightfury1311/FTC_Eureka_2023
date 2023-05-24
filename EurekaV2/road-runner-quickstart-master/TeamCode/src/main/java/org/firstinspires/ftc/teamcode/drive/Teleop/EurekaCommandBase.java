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


import org.firstinspires.ftc.teamcode.drive.CommandBase.ArmHome;
import org.firstinspires.ftc.teamcode.drive.CommandBase.ArmPick;
import org.firstinspires.ftc.teamcode.drive.CommandBase.ArmDrop;
import org.firstinspires.ftc.teamcode.drive.CommandBase.ElevatorHigh;

import org.firstinspires.ftc.teamcode.drive.CommandBase.ElevatorHome;
import org.firstinspires.ftc.teamcode.drive.CommandBase.ElevatorLow;
import org.firstinspires.ftc.teamcode.drive.CommandBase.ElevatorMid;
import org.firstinspires.ftc.teamcode.drive.CommandBase.GripperClose;
import org.firstinspires.ftc.teamcode.drive.CommandBase.GripperOpen;
import org.firstinspires.ftc.teamcode.drive.CommandBase.GripperRotatePick;
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Drive;
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Elevator;
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Slide;
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Servos;

import java.util.function.BooleanSupplier;
import  com.acmerobotics.dashboard.config.Config;


@Config
@TeleOp(name = "EurekaCMD")
public class EurekaCommandBase extends CommandOpMode {

    Elevator elevator;
    Drive drive;

    Slide slide;
    Servos serv;
    GamepadEx driver;
    Button  liftHighButton, liftMidButton, liftLowButton, liftHomeButton, gripperButton, homeButton, pickButton;
    @Override
    public void initialize() {

        driver = new GamepadEx(gamepad1);

        liftHighButton = new GamepadButton(driver, GamepadKeys.Button.DPAD_UP);
        liftMidButton = new GamepadButton(driver, GamepadKeys.Button.Y);
        liftLowButton = new GamepadButton(driver, GamepadKeys.Button.A);
        liftHomeButton = new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN);
        gripperButton = new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER);
        homeButton = new GamepadButton(driver, GamepadKeys.Button.BACK);
        pickButton = new GamepadButton(driver, GamepadKeys.Button.DPAD_LEFT);

        this.elevator = new Elevator(hardwareMap, telemetry);
        this.serv = new Servos(hardwareMap, telemetry);
        this.drive = new Drive(hardwareMap, telemetry, this);

        liftHighButton.whenPressed(new ParallelCommandGroup(new ElevatorHigh(elevator)));
        liftMidButton.whenPressed(new ParallelCommandGroup(new ElevatorMid(elevator)));
        liftLowButton.toggleWhenPressed(new ParallelCommandGroup(new ElevatorLow(elevator)), (new ElevatorHome(elevator)));
        liftHomeButton.whenPressed(new ParallelCommandGroup(new ElevatorHome(elevator)));
        gripperButton.toggleWhenPressed(new ParallelCommandGroup(new GripperClose(serv)), (new GripperOpen(serv)));
//        homeButton.whenPressed(new ParallelCommandGroup(new ElevatorHome(elevator), new GripperRotatePick(serv), new GripperOpen(serv), new ArmHome(serv)));
        pickButton.whenPressed(new ParallelCommandGroup(new ArmPick(serv)));
        register(drive);

            }
        }



