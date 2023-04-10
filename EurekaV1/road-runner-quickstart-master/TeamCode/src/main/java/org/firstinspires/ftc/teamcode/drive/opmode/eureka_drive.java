package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.drive.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Disabled
@TeleOp
public class eureka_drive extends LinearOpMode {
//    double kp_gyro = 0.04;
//    double kd_gyro = 0.01;
//    double ki_gyro = 0;
//    double prevError_gyro = 0;

    double kp_pole_mid = 0.04;
    double kd_pole_mid = 0.05;
    double ki_pole_mid = 0;
    double prevError_pole_mid = 0;

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Sensors sensors = new Sensors(hardwareMap, telemetry);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();


        while (opModeIsActive()) {
            Pose2d poseEstimate = drive.getPoseEstimate();
            double heading = Math.toDegrees(angleWrap(poseEstimate.getHeading()));
//            double headingPower = gyroPID(heading, 0);

            double distanceRight = Sensors.PoleRight.getDistanceCM();
            double distanceMid = Sensors.PoleMid.getDistanceCM();
            double distanceLeft = Sensors.PoleLeft.getDistanceCM();
            double polePowerMid = 0;


             if (gamepad1.y) {
                polePowerMid = polePIDMid(distanceMid, 12.5);
            }
             else{
                polePowerMid = 0;
            }
            if (gamepad1.back) {

                double diff = (distanceLeft - distanceRight) / 2.0;

                if (diff < 0) {
                    drive.setWeightedDrivePower(new Pose2d(0,0.1,0));
                } else {
                    drive.setWeightedDrivePower(new Pose2d(0,-0.1,0));
                }
            }
            drive.setWeightedDrivePower(new Pose2d(polePowerMid,0.4,0));

            drive.update();
            telemetry.addData("RightSensor is at: ", distanceRight + "CM");
            telemetry.addData("MidSensor is at: ", distanceMid + "CM");
            telemetry.addData("LeftSensor is at: ", distanceLeft + "CM");
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("heading normalized", heading);
            telemetry.update();
        }
    }

//    public double gyroPID(double current, double target) {
//        double error = current - target;
//        double pError = error;
//        double dError = error - prevError_gyro;
//        double Ierror = error + prevError_gyro;
//
//        prevError_gyro = error;
//
//        return pError * kp_gyro + dError * kd_gyro + Ierror * ki_gyro;
//    }

    public double polePIDMid(double current, double target) {
        double error = current - target;
        double pError = error;
        double dError = error - prevError_pole_mid;
        double Ierror = error + prevError_pole_mid;

        prevError_pole_mid = error;

        return pError * kp_pole_mid + dError * kd_pole_mid + Ierror * ki_pole_mid;
    }


    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }
}

