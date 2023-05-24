package org.firstinspires.ftc.teamcode.drive.CommandBase;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Servos;


public class ArmDrop extends CommandBase {

    public Servos serv;


    public ArmDrop(Servos serv){
        this.serv = serv;
        addRequirements(serv);
    }

    @Override
    public void initialize() {
        super.initialize();
        Servos.Arm.goDrop();
        Servos.Arm.goActiveDrop();
        Servos.Rotate.rotateDrop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
