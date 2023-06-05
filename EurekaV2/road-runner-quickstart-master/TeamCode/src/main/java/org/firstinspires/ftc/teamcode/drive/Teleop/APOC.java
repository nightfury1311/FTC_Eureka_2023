//package org.firstinspires.ftc.teamcode.drive.Teleop;
//
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//
//import org.firstinspires.ftc.teamcode.drive.Robot;
//import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Elevator;
//import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Servos;
//import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.SlidersEx;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//
//@Config
//@TeleOp
//public class APOC extends LinearOpMode {
//
//
//    public static long a = 500;
//
//    boolean RB1Flag = false;
//    boolean LB1Flag = false;
//    boolean B1Flag = false;
//    boolean AFlag1=false;
//    boolean XFlag1=false;
//    boolean  STARTFLAG1=false;
//
//
//    boolean XFlag2 = false;
//    boolean AFlag2 = false;
//    boolean RB2Flag = false;
//    boolean BACKFlag2 = false;
//
//    boolean flagForSliderHome = false;
//
//    public boolean isDelayComplete = false;
//    public double temp ;
//    public double THROTTLE;
//    public double TURN;
//    public double HEADING;
//    public static double MAXVEL = 2000;
//    public static double MAXACC = 2000;
//
//
//    Elevator elevator = null;
//    Servos servos = null;
//    SlidersEx slide = null;
//
//    Robot drive = null;
////    Timers timer = null;
//    ElapsedTime t = null;
//
//    public static int elevatorPos = 0;
//    public static int sliderPos = 0;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        elevator = new Elevator(hardwareMap, telemetry);
//        slide = new SlidersEx(hardwareMap, telemetry);
//        servos = new Servos(hardwareMap, telemetry);
//        drive = new Robot(hardwareMap, telemetry, slide);
//
//        t = new ElapsedTime();
//
//
//
//        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
//        drive.setPoseEstimate(startPose);
//
//        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
//
//
//        while (opModeInInit()) {
//
//            isDelayComplete = false;
//
//            elevator.reset();
//            slide.reset();
//            Servos.Gripper.Lock();
//            Servos.Gripper.openGripper();
//            Servos.Arm.goActiveStable();
//            Servos.Arm.goInit();
//            Servos.Rotate.rotatePick();
//
//        }
//
//
//        elevatorPos = 0;
//        sliderPos = 0;
//
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//
//            // driver
//            boolean A1 = gamepad1.a;
//            boolean B1 = gamepad1.b;
//            boolean X1 = gamepad1.x;
//            boolean Y1 = gamepad1.y;
//            boolean UP1 = gamepad1.dpad_up;
//            boolean RIGHT1 = gamepad1.dpad_right;
//            boolean DOWN1 = gamepad1.dpad_down;
//            boolean LEFT1 = gamepad1.dpad_left;
//            boolean RB1 = gamepad1.right_bumper;
//            boolean LB1 = gamepad1.left_bumper;
//            boolean START1 = gamepad1.start;
//            boolean BACK1 = gamepad1.back;
//            double RTG1 = gamepad1.right_trigger;
//            double LTG1 = gamepad1.left_trigger;
//
//
//            Pose2d poseEstimate = drive.getPoseEstimate();
//            Vector2d input = new Vector2d(Math.pow(Range.clip(gamepad1.left_stick_y, -1, 1), 3), Math.pow(Range.clip(gamepad1.left_stick_x, -1, 1), 3)).rotated(-poseEstimate.getHeading());
//            drive.setWeightedDrivePower(new Pose2d(input.getX() * THROTTLE, input.getY() * TURN, -gamepad1.right_stick_x * HEADING));
//            drive.update();
//
//
//            // TODO : SLOW MODE - DRIVE
//            if (LTG1 > 0.7) {    // slow mode
//                THROTTLE = 0.4;
//                TURN = 0.4;
//                HEADING = 0.2;
//            } else {
//                THROTTLE = 0.9;
//                TURN = 0.9;
//                HEADING = 0.6;
//            }
//
//
//            // TODO : Everything to Init pos
//            if(BACK1){
//
//            }
//
//            // TODO : gripper open-CLOSE
//            if (A1 && !AFlag1) {
//                AFlag1 = true;
//                telemetry.addLine("GRIPPPPERRR");
//                if (Servos.Gripper.gripperState == "OPEN") {
//                    Servos.Gripper.openGripper();
//                    telemetry.addLine("GRIPPER CLOSE");
//                } else if (Servos.Gripper.gripperState == "CLOSE") {
//                    Servos.Gripper.closeGripper();
//                    telemetry.addLine("GRIPPER OPEN");
//                }
//            }
//            if (!A1) {
//                AFlag1 = false;
//            }
//
//            if(B1){
//
//            }
//
//
//            // TODO : Elevator home position
//            if(DOWN1){
//                elevator.extendToHomePos();
//               Servos.Gripper.Unlock();
//            }
//
//            // TODO : ELEVATOR HIGH
//            if(RIGHT1){
//                elevator.extendToHighPole();
//            }
//
//            // TODO : ELEVATOR MEDIUM
//            if (UP1){
//                elevator.extendToMidPole();
//            }
//
//            // TODO : ELEVATOR LOW
//            if (UP1){
//                elevator.extendToLowPole();
//            }
//
//            // TODO : ARM AND END EFFECTOR TO LOW POLE
//            if(LEFT1){
//
//            }
//
//            // TODO : STACKING ON HIGH
//
//            if (Y1){
//
//            }
//
//            //TODO  ARM POSITION - INIT AND GRIPPING
//
//            if(X1 && !XFlag1){
//                XFlag1 = true;
//                if((Arm.armState == "INIT" || Arm.armState == "LOWPOLE") && (EndEffector.yawState == "INIT" || EndEffector.yawState == "LOWPOLE") && (EndEffector.wristState == "INIT" || EndEffector.wristState == "LOWPOLE")){
//                    Arm.GrippingPos();
//                    EndEffector.Wrist.GrippingPos();
//                    EndEffector.Yaw.GrippingPos();
//
//                    telemetry.addLine("Arm in gripping position");
//                    telemetry.update();
//                }
//                else if ((Arm.armState == "GRIP" || Arm.armState == "PLACE") && (EndEffector.yawState == "GRIP" || EndEffector.yawState == "PLACE")&& (EndEffector.wristState == "GRIP" || EndEffector.wristState == "PLACE")) {
//                    Arm.InitPos();
//                    EndEffector.Wrist.InitPos();
//                    EndEffector.Yaw.InitPos();
//
//                    telemetry.addLine("Arm in init position");
//                    telemetry.update();
//                }
//            }
//            if(!X1){
//                XFlag1 = false;
//            }
//
//            if(RB1 && !RB1Flag){
//                RB1Flag = true;
//                if (coneLocking.lockState == "LOCK"){
//                    ConeLocking.coneUnlock();
//                    telemetry.addLine("Cone Lock open");
//                    telemetry.update();
//                }
//                else if (coneLocking.lockState == "UNLOCK"){
//                    ConeLocking.coneLock();
//                    telemetry.addLine("Cone Lock shut");
//                    telemetry.update();
//                }
//            }
//
//            if (!RB1){
//                RB1Flag = false;
//            }
//
//
//
//            // TODO : TOGGLE - SLIDER OPEN ---> GRIPPING POS ----> GRIP ---> ARM TO PLACING POS ----> LOCK ----> ARM TO INIT POS
//
//            if(LB1 && !LB1Flag){
//                LB1Flag = true;
//
//                if(Slider.sliderState == "CLOSE"){
//                    slider.goTo(800);
//                    EndEffector.Gripper.gripperOpen();
//                    EndEffector.Yaw.GrippingPos();
//                    Arm.GrippingPos();
//                    ConeLocking.coneUnlock();
//                    EndEffector.Wrist.GrippingPos();
//
//                }
//
//                else if(Slider.sliderState == "OPEN" ){
//
//                    drive.followTrajectorySequenceAsync(traj1);
//                    drive.update();
//
//                }
//            }
//
//            if (!LB1){
//                LB1Flag = false;
//            }
//
//            drive.update();
//            telemetry.update();
//
//
//
//            // TODO on left click reset heading
//
//            if(gamepad1.left_stick_button){
//                drive.setPoseEstimate(startPose);
//            }
//
//        }
//
//    }
//
//}
//
