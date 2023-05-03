//package org.firstinspires.ftc.teamcode.drive.Teleop;
//
//import com.acmerobotics.roadrunner.profile.MotionProfile;
//import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
//import com.acmerobotics.roadrunner.profile.MotionState;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//public class GPT extends LinearOpMode {
//    // Motor objects
//    private DcMotor motor1;
//    private DcMotor motor2;
//
//    // Encoder values
//    private int currentPosition;
//    private int targetPosition;
//
//    // Motion profile constants
//    private static final double MAX_VELOCITY = 500.0;
//    private static final double MAX_ACCELERATION = 1000.0;
//
//    // Constructor
//    public GPT(DcMotor motor1, DcMotor motor2) {
//        this.motor1 = motor1;
//        this.motor2 = motor2;
//        this.currentPosition = 0;
//        this.targetPosition = 0;
//    }
//
//    // Move to a given position
//    public void moveToPosition(int position) {
//        this.targetPosition = position;
//        // Calculate the motion profile
//        MotionProfile profile = generateMotionProfile(currentPosition, targetPosition);
//
//        // Execute the motion profile
//        for (MotionState state : profile.getStates()) {
//            setMotorsPower(state.getVelocity() / MAX_VELOCITY);
//            sleep(state.getDuration());
//        }
//        // Update the current position
//        currentPosition = targetPosition;
//    }
//
//    // Generate a motion profile
//    private MotionProfile generateMotionProfile(int startPosition, int targetPosition) {
//        MotionProfileGenerator generator = new MotionProfileGenerator(MAX_VELOCITY, MAX_ACCELERATION);
//        return generator.generateProfile(startPosition, targetPosition);
//    }
//
//    // Set the power of both motors
//    private void setMotorsPower(double power) {
//        motor1.setPower(power);
//        motor2.setPower(power);
//    }
//
//    // Sleep for a given duration in milliseconds
//    public void sleep(long duration) {
//        try {
//            Thread.sleep(duration);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
//    }
//}
//
