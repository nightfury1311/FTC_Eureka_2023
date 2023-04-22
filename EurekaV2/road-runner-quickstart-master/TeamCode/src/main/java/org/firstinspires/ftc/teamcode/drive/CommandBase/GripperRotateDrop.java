package org.firstinspires.ftc.teamcode.drive.CommandBase;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Servos;


public class GripperRotateDrop extends CommandBase {

    public Servos servos;


    public GripperRotateDrop(Servos servos){
        this.servos = servos;
        addRequirements(servos);
    }

    @Override
    public void initialize() {
        super.initialize();
        Servos.Rotate.rotateDrop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
