package org.firstinspires.ftc.teamcode.drive.CommandBase;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Servos;


public class GripperRotatePick extends CommandBase {

    public Servos serv;


    public GripperRotatePick(Servos serv){
        this.serv = serv;
        addRequirements(serv);
    }

    @Override
    public void initialize() {
        super.initialize();
        Servos.Rotate.rotatePick();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
