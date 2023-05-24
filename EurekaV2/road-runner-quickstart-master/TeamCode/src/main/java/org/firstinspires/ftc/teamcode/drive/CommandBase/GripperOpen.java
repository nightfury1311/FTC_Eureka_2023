package org.firstinspires.ftc.teamcode.drive.CommandBase;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Servos;


public class GripperOpen extends CommandBase {

        public Servos servos;


    public GripperOpen(Servos servos){
        this.servos = servos;
        addRequirements(servos);
    }

    @Override
    public void initialize() {
        super.initialize();
        Servos.Gripper.openGripper();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
