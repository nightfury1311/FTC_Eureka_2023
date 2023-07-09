package org.firstinspires.ftc.teamcode.drive.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Servos;
@Disabled
@TeleOp
public class ServoStack extends LinearOpMode {
    Servos servos = null;

    @Override

    public void runOpMode() {
        servos = new Servos(hardwareMap, telemetry);


        while (opModeInInit()) {
            Servos.Gripper.openGripper();
            Servos.Arm.goActiveStable();
            Servos.Arm.goInit();
            Servos.Rotate.rotatePick();
        }
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                Servos.Rotate.rotatePick();
                Servos.Gripper.openGripper();
                sleep(300);
                Servos.Arm.goActivePick1();
                Servos.Arm.goPickCone1();
            } else if (gamepad1.dpad_right) {
                Servos.Rotate.rotatePick();
                Servos.Gripper.openGripper();
                sleep(300);
                Servos.Arm.goActivePick2();
                Servos.Arm.goPickCone2();
            }
            else if (gamepad1.dpad_down) {
                Servos.Rotate.rotatePick();
                Servos.Gripper.openGripper();
                sleep(300);
                Servos.Arm.goActivePick3();
                Servos.Arm.goPickCone3();
            }
            else if (gamepad1.dpad_left) {
                Servos.Rotate.rotatePick();
                Servos.Gripper.openGripper();
                sleep(300);
                Servos.Arm.goActivePick4();
                Servos.Arm.goPickCone4();
            }
            else if (gamepad1.left_bumper) {
                Servos.Rotate.rotatePick();
                Servos.Gripper.openGripper();
                sleep(300);
                Servos.Arm.goActivePick();
                Servos.Arm.goPickTele();
            } else if (gamepad1.start) {

                Servos.Gripper.closeGripper();
                sleep(300);
                Servos.Arm.goActivePick1();
                Servos.Arm.goInit();
                sleep(500);
                Servos.Rotate.rotateDrop();
                sleep(300);
                Servos.Arm.goActiveDrop();

                Servos.Arm.goDrop();

                sleep(800);

                Servos.Gripper.openGripper();
                sleep(200);
                Servos.Arm.goInit();
                sleep(200);
                Servos.Gripper.Lock();
                sleep(200);
                Servos.Rotate.rotatePick();
                Servos.Arm.goActiveStable();
            }
        }
    }
}
