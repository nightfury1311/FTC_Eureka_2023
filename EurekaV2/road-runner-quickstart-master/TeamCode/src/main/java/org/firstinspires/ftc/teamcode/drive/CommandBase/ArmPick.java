package org.firstinspires.ftc.teamcode.drive.CommandBase;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Servos;


public class ArmPick extends CommandBase {

    public Servos serv;


    public ArmPick(Servos serv){
        this.serv = serv;
        addRequirements(serv);
    }

    @Override
    public void initialize() {
        super.initialize();
        Servos.Arm.goPick();
        Servos.Arm.goActivePick();
        Servos.Rotate.rotatePick();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
