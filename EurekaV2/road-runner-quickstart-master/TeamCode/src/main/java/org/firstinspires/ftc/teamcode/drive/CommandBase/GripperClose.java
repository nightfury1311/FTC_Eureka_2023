package org.firstinspires.ftc.teamcode.drive.CommandBase;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Servos;


public class GripperClose extends CommandBase {

    public Servos servos;


    public GripperClose(Servos servos){
        this.servos = servos;
        addRequirements(servos);
    }

    @Override
    public void initialize() {
        super.initialize();
        Servos.Gripper.closeGripper();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
