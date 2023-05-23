//package org.firstinspires.ftc.teamcode.drive.Teleop;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.ConditionalCommand;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.arcrobotics.ftclib.command.button.Button;
//import com.arcrobotics.ftclib.command.button.GamepadButton;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//
//import org.firstinspires.ftc.teamcode.drive.CommandBase.ElevatorHigh;
//
//import org.firstinspires.ftc.teamcode.drive.CommandBase.ElevatorHome;
//import org.firstinspires.ftc.teamcode.drive.CommandBase.ElevatorLow;
//import org.firstinspires.ftc.teamcode.drive.CommandBase.ElevatorMid;
//import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Drive;
//import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Elevator;
//import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Servos;
//
//import java.util.function.BooleanSupplier;
//import  com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.hardware.motors.RevRoboticsUltraPlanetaryHdHexMotor;
//import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//@Config
//@TeleOp(name = "EurekaCMD")
//public class EurekaCommandBase extends CommandOpMode {
//
//    Elevator elevator;
//    Drive drive;
//    GamepadEx driver;
//    Button  liftHighButton, liftMidButton, liftLowButton, liftHomeButton, gripperButton;
//    @Override
//    public void initialize() {
//
//        driver = new GamepadEx(gamepad1);
//
//        liftHighButton = new GamepadButton(driver, GamepadKeys.Button.DPAD_UP);
//        liftMidButton = new GamepadButton(driver, GamepadKeys.Button.Y);
//        liftLowButton = new GamepadButton(driver, GamepadKeys.Button.A);
//        liftHomeButton = new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN);
//        gripperButton = new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER);
//
//        this.elevator = new Elevator(hardwareMap, telemetry);
//        this.drive = new Drive(hardwareMap, telemetry, this);
//
//        liftHighButton.whenPressed(new ParallelCommandGroup(new ElevatorHigh(elevator)));
//        liftMidButton.whenPressed(new ParallelCommandGroup(new ElevatorMid(elevator)));
//        liftLowButton.toggleWhenPressed(new ParallelCommandGroup(new ElevatorLow(elevator)), (new ElevatorHome(elevator)));
//        liftHomeButton.whenPressed(new ParallelCommandGroup(new ElevatorHome(elevator)));
//        register(drive);
//            }
//        }
//
//
//
