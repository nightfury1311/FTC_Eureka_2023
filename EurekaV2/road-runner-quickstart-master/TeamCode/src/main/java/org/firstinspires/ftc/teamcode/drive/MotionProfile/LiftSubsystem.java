package org.firstinspires.ftc.teamcode.drive.MotionProfile;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.MotionProfile.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.drive.MotionProfile.Constraints;
import org.firstinspires.ftc.teamcode.drive.MotionProfile.State;
import org.firstinspires.ftc.teamcode.drive.MotionProfile.Globals;
import org.firstinspires.ftc.teamcode.drive.MotionProfile.elevator_sub;


@Config
public class LiftSubsystem extends SubsystemBase {

    // stores the state of the subsystem
    // anything other than GOOD means it goofed
    private elevator_sub elevator_sub;
//    public LiftSubsystem.STATE state = LiftSubsystem.STATE.GOOD;
    public LiftState liftState = LiftState.RETRACTED;

    private AsymmetricMotionProfile liftProfile;
    public State liftMotionState;
    private final ElapsedTime timer;
    private PIDController controller;

    private int liftPosition;
    private double power = 0.0;
    private int targetPosition = 0;


    public static double P = 0.01;
    public static double I = 0.1;
    public static double D = 0.000125;
    public static double F = 0.1;


    public enum LiftState {
        HIGH,
        RETRACTED
    }

    public LiftSubsystem(elevator_sub elevator_sub) {
        this.elevator_sub = elevator_sub;

        this.liftProfile = new AsymmetricMotionProfile(0, 1, new Constraints(0, 0, 0));
        this.controller = new PIDController(P, I, D);
        this.timer = new ElapsedTime();

    }


    public void update(LiftState state) {
        liftState = state;
        switch (state) {
            case HIGH:
                elevator_sub.extendTo(elevator_sub.POSITIONS[elevator_sub.HIGH_POLE]);
//                newProfile(LIFT_HIGH_POS, new Constraints(LIFT_MAX_V, LIFT_MAX_A, LIFT_MAX_D));
                break;
            case RETRACTED:
                elevator_sub.extendTo(elevator_sub.POSITIONS[elevator_sub.HOME]);
//                newProfile(LIFT_RETRACT_POS, new Constraints(LIFT_MAX_V, LIFT_MAX_A, LIFT_MAX_D));
                break;
        }
    }

    public void loop() {
        this.controller.setPID(P, I, D);

        liftMotionState = liftProfile.calculate(timer.time());
        if (liftMotionState.v != 0) {
            setTargetPos((int) liftMotionState.x);
        }

        power = Range.clip((-controller.calculate(liftPosition, targetPosition) + 0.14), -0.5, 0.5);
    }

    public void write() {
        try {
            elevator_sub.leftMotor.set(power);
            elevator_sub.rightMotor.set(power);
        } catch (Exception e) {}
    }

    public double getPos() {
        return liftPosition;
    }

    public void setTargetPos(int targetPosition) {
        this.targetPosition = targetPosition;
    }

    public int getTargetPos() {
        return targetPosition;
    }

    public void resetTimer() {
        timer.reset();
    }

    public void newProfile(double targetPos, Constraints constraints) {
        constraints.convert(22);
        this.liftProfile = new AsymmetricMotionProfile(getPos(), targetPos, constraints);
        resetTimer();
    }
}
