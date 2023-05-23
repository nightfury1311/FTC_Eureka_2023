//package org.firstinspires.ftc.teamcode.drive.CommandBase;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//
//import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Elevator;
//
//public class ElevatorMid extends CommandBase {
//
//    public  Elevator elevator;
//
//    public ElevatorMid(Elevator elevator){
//        this.elevator = elevator;
//        addRequirements(elevator);
//    }
//
//    @Override
//    public void initialize() {
//        super.initialize();
//        elevator.extendToMidPole();
//    }
//
//    @Override
//    public boolean isFinished() {
//        return true;
//    }
//}