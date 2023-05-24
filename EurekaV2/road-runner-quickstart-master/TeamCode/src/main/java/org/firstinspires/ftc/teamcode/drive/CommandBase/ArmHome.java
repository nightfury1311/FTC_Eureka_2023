package org.firstinspires.ftc.teamcode.drive.CommandBase;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Servos;


public class ArmHome extends CommandBase {

    public Servos serv;


    public ArmHome(Servos serv){
        this.serv = serv;
        addRequirements(serv);
    }

    @Override
    public void initialize() {
        super.initialize();
        Servos.Arm.goInit();
        Servos.Arm.goActiveStable();
        Servos.Rotate.rotatePick();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
