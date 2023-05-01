//package org.firstinspires.ftc.teamcode.drive.Teleop;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.Range;
//
//public class GPT extends OpMode {
//
//    private DcMotor elevatorLeft;
//    private DcMotor elevatorRight;
//
//    // The target position for the elevator
//    private double targetPosition = 0;
//
//    // The maximum speed of the elevator
//    private double maxSpeed = 1.0;
//
//    // The maximum acceleration of the elevator
//    private double maxAcceleration = 0.5;
//
//    // The time elapsed since the elevator started moving
//    private double elapsedTime = 0;
//
//    // The position of the elevator at the start of the motion profiling
//    private double initialPosition = 0;
//
//    // The speed of the elevator at the start of the motion profiling
//    private double initialSpeed = 0;
//
//    // The current position of the elevator
//    private double currentPosition = 0;
//
//    // The current speed of the elevator
//    private double currentSpeed = 0;
//
//    // The current acceleration of the elevator
//    private double currentAcceleration = 0;
//
//    // The time of the last loop iteration
//    private double lastLoopTime = 0;
//
//    @Override
//    public void init() {
//        elevatorLeft = hardwareMap.dcMotor.get("elevatorLeft");
//        elevatorRight = hardwareMap.dcMotor.get("elevatorRight");
//        elevatorRight.setDirection(DcMotor.Direction.REVERSE);
//    }
//
//    @Override
//    public void loop() {
//        double loopTime = getRuntime() - lastLoopTime;
//        lastLoopTime = getRuntime();
//
//        // Calculate the new position and speed of the elevator
//        double deltaPosition = currentSpeed * loopTime + 0.5 * currentAcceleration * loopTime * loopTime;
//        currentPosition = initialPosition + deltaPosition;
//        currentSpeed = initialSpeed + currentAcceleration * loopTime;
//
//        // Calculate the new acceleration of the elevator
//        double deltaSpeed = targetSpeed - currentSpeed;
//        double maxDeltaSpeed = maxAcceleration * loopTime;
//        double accelerationSign = Math.signum(deltaSpeed);
//        double deltaAcceleration = accelerationSign * Range.clip(Math.abs(deltaSpeed), 0, maxDeltaSpeed);
//        currentAcceleration = Range.clip(initialAcceleration + deltaAcceleration, -maxAcceleration, maxAcceleration);
//
//        // Set the motor power based on the current speed of the elevator
//        double motorPower = Range.clip(currentSpeed / maxSpeed, -1, 1);
//        elevatorLeft.setPower(motorPower);
//        elevatorRight.setPower(motorPower);
//
//        // Update the elapsed time and reset the motion profiling if the elevator has reached the target position
//        elapsedTime += loopTime;
//        if (elapsedTime >= targetTime) {
//            initialPosition = currentPosition;
//            initialSpeed = currentSpeed;
//            initialAcceleration = currentAcceleration;
//            elapsedTime = 0;
//        }
//    }
//
//    // Sets the target position for the elevator and calculates the target time
//    public void setTargetPosition(double position) {
//        targetPosition = position;
//        double deltaPosition = targetPosition - currentPosition;
//        targetTime = Math.sqrt(Math.abs(2 * deltaPosition / maxAcceleration));
//    }
//}
//
