package org.firstinspires.ftc.teamcode.drive.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TeleopSubsystems.Servos;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@TeleOp(name = "APOC")
public class APOC extends LinearOpMode {

    Servos servos = null;

    private PIDController elevatorcontroller;
    private PIDController slidercontroller;
    public static double elevatorp = 0.006, elevatori = 0, elevatord = 0, elevatorff = -0.14;
    public static double sliderp = 0.008, slideri = 0, sliderd = 0, sliderff = -0.2;

    public static int elevatortarget = 0;
    public static int HIGH = 660;   // HIGH POLE
    public static int MID = 410;   // MID POLE
    public static int LOW = 183;   // LOW POLE
    public static int slidertarget = 0;

    public static int MAX = 600; //Slider long extension
    public static int TEST = 275;   // slider small extension
    public static int HOME = 0;     // slider + elevator home

    private DcMotorEx ElevateLeft, ElevateRight;
    private DcMotorEx SlideLeft, SlideRight;

    public static double speed = 1;
    public static double turn = 0.6;

    // Flags for toggle commands

    int RB1Flag = 0;

    boolean RTG1Flag = false; // Gripping

    boolean X1Flag = false; // Ground junction placement + retraction

    boolean B1Flag = false; // Locking

    boolean START1Flag = false;  // Cycle Flag
    @Override
    public void runOpMode() throws InterruptedException {

        servos = new Servos(hardwareMap, telemetry);
        elevatorcontroller = new PIDController(elevatorp,elevatori,elevatord);
        slidercontroller= new PIDController(sliderp,slideri,sliderd);

        telemetry= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ElevateLeft = hardwareMap.get(DcMotorEx.class, "ElevateLeft");
        ElevateRight = hardwareMap.get(DcMotorEx.class, "ElevateRight");
        SlideLeft = hardwareMap.get(DcMotorEx.class, "SlideLeft");
        SlideRight = hardwareMap.get(DcMotorEx.class, "SlideRight");
        ElevateLeft.setDirection(DcMotorEx.Direction.REVERSE);
        ElevateRight.setDirection(DcMotorEx.Direction.REVERSE);
        SlideRight.setDirection(DcMotorSimple.Direction.REVERSE);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, telemetry);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        telemetry= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        while ((opModeInInit())) {
            // Initialization - resetting all motors and servos
            Servos.Gripper.Unlock();
            Servos.Gripper.openGripper();
            Servos.Arm.goActiveStable();
            Servos.Arm.goInit();
            Servos.Rotate.rotatePick();
        }

        TrajectorySequence transfer = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(()->{Servos.Gripper.Unlock();})
                .addTemporalMarker(()->{slidertarget=MAX+50;})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{Servos.Gripper.closeGripper();})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{Servos.Arm.goInit();Servos.Arm.goActiveStable();slidertarget=HOME;})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Servos.Arm.goActiveDrop();Servos.Rotate.rotateDrop();})
                .waitSeconds(0.4)
                .addTemporalMarker(()->{Servos.Arm.goDrop();})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Servos.Gripper.openGripper();})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{Servos.Gripper.Lock();Servos.Arm.goInit();})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{Servos.Rotate.rotatePick();Servos.Arm.goActiveStable();})
                .build();


        TrajectorySequence cycle = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(()->{slidertarget=TEST+100;})
                .waitSeconds(0.05)
                .addTemporalMarker(()->{Servos.Gripper.closeGripper();})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{Servos.Arm.goInit();Servos.Arm.goActiveStable();slidertarget=HOME;})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Servos.Arm.goActiveDrop();Servos.Rotate.rotateDrop();})
                .waitSeconds(0.4)
                .addTemporalMarker(()->{Servos.Arm.goDrop();})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Servos.Gripper.openGripper();})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{Servos.Gripper.Lock();Servos.Arm.goPickTele();})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{Servos.Rotate.rotatePick();Servos.Arm.goActivePick();})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{elevatortarget=HIGH;})
                .waitSeconds(0.7)
                .addTemporalMarker(()->{slidertarget=TEST-200;})
                .addTemporalMarker(()->{elevatortarget=HOME;Servos.Gripper.Unlock();})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{slidertarget=TEST;})
                .build();


        while (opModeIsActive()) {
            elevatorcontroller.setPID(elevatorp, elevatori, elevatord);
            slidercontroller.setPID(sliderp, slideri, sliderd);

            int ElevateFinalPos = ElevateRight.getCurrentPosition();
            int SlideFinalPos = SlideRight.getCurrentPosition();
            double elevatorpower = Range.clip(((elevatorcontroller.calculate(ElevateFinalPos, elevatortarget)+ elevatorff)) , -0.9, 1);
            double sliderpower =  Range.clip(((slidercontroller.calculate(SlideFinalPos, slidertarget)+ sliderff)) , -0.8, 0.9);
            ElevateLeft.setPower(elevatorpower);
            ElevateRight.setPower(elevatorpower);
            SlideLeft.setPower(sliderpower);
            SlideRight.setPower(sliderpower);

            telemetry.addData("Status", "Running");

            //Field centric drive
            Pose2d poseEstimate = drive.getPoseEstimate();
            Vector2d input = new Vector2d(Math.pow(Range.clip(-gamepad1.left_stick_y, -1, 1), 3), Math.pow(Range.clip(-gamepad1.left_stick_x,-1,1), 3)).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(new Pose2d(input.getX() * speed, input.getY() * speed, -gamepad1.right_stick_x * turn));
            drive.update();

            // GAMEPAD FUNCTIONALITY
            boolean A1 = gamepad1.a;                        // Elevator - Low junction
            boolean B1 = gamepad1.b;                        // Toggling lock state
            boolean X1 = gamepad1.x;                        // Ground junction
            boolean Y1 = gamepad1.y;                        // Elevator - Mid junction
            boolean UP1 = gamepad1.dpad_up;                 // Elevator - High junction
            boolean RIGHT1 = gamepad1.dpad_right;           // Cone pickup - Low junction
            boolean DOWN1 = gamepad1.dpad_down;             // Elevator -  Home position
            boolean LEFT1 = gamepad1.dpad_left;             // Cone pickup - Without extension
            boolean RB1 = gamepad1.right_bumper;            // Cone transfer
            boolean LB1 = gamepad1.left_bumper;             // Cone pickup - With extension
            boolean START1 = gamepad1.start;                // High junction cycle
            boolean BACK1 = gamepad1.back;                  // Resetting all motors and servos / Home position
            boolean LStick = gamepad1.left_stick_button;    // Resetting heading
            boolean RStick = gamepad1.right_stick_button;   // Resetting heading
            double RTG1 = gamepad1.right_trigger;           // Toggling gripper state
            double LTG1 = gamepad1.left_trigger;            // Slowmode


            // Defining commands/keybinds

            if (LStick || RStick) {         // RESETTING HEADING (FOR FIELD CENTRIC)
                drive.setPoseEstimate(startPose);
            }

            if (BACK1) {                    // EVERYTHING IN HOME POSITION

                Servos.Arm.goActiveStable();
                Servos.Arm.goInit();
                Servos.Rotate.rotatePick();
                elevatortarget=HOME;
                slidertarget=HOME;

            }

            if (LTG1 > 0.7 || (Servos.Arm.armState  == "PICKTELE" && Servos.Gripper.gripperState == "CLOSE") )    // SLOW MODE
            {
                speed = 0.4;
                turn = 0.2;
            } else {
                speed = 1;
                turn = 0.6;
            }


            if (RTG1 > 0.7 && !RTG1Flag) {       // GRIPPER OPEN CLOSE
                RTG1Flag = true;
                if (Servos.Gripper.gripperState == "CLOSE") {   // If closed, then open
                    Servos.Gripper.openGripper();
                } else if (Servos.Gripper.gripperState == "OPEN") {     // If open, then close
                    Servos.Gripper.closeGripper();
                }
            }

            if (LB1) {      // GOTO PICK CONES WITH SLIDERS
                Servos.Gripper.Unlock();
                Servos.Arm.goActivePick();
                Servos.Rotate.rotatePick();
                Servos.Gripper.openGripper();
                sleep(300);
                Servos.Arm.goPickTele();
                sleep(300);
                slidertarget=MAX;
            }

            if (RB1 && RB1Flag == 0) {
                if (SlideFinalPos > 200) {     // CONE TRANSFERS
                    RB1Flag = 1;
                    drive.followTrajectorySequenceAsync(transfer);
                }
                else if (SlideFinalPos < 100 && Servos.Arm.armState  == "PICKTELE") {   // Picking without extension
                    RB1Flag = 2;
                    Servos.Gripper.Unlock();
                    Servos.Gripper.closeGripper();
                    sleep(200);
                    Servos.Rotate.rotateDrop();
                    Servos.Arm.goActiveDrop();
                    Servos.Arm.goDrop();
                    sleep(800);
                    Servos.Gripper.openGripper();
                    sleep(100);
                    Servos.Arm.goInit();
                    Servos.Gripper.Lock();
                    sleep(100);
                    Servos.Rotate.rotatePick();
                    Servos.Arm.goActiveStable();
                }
                else if (Servos.Arm.armState == "ActiveLow" || Servos.Arm.armState == "ActiveStable") {   // Dropping from arm home position
                    RB1Flag = 3;
                    Servos.Gripper.Unlock();
                    Servos.Rotate.rotateDrop();
                    Servos.Arm.goActiveDrop();
                    sleep(200);
                    Servos.Arm.goDrop();
                    sleep(300);
                    Servos.Gripper.openGripper();
                    sleep(100);
                    Servos.Gripper.Lock();
                    Servos.Arm.goInit();
                    sleep(100);
                    Servos.Rotate.rotatePick();
                    Servos.Arm.goActiveStable();
                }
            }

            if (B1 && !B1Flag) {     //CONE LOCK TOGGLE
                B1Flag = true;
                if (Servos.Gripper.lockState == "UNLOCK") {
                    Servos.Gripper.Lock();
                } else if (Servos.Gripper.lockState == "LOCK") {
                    Servos.Gripper.Unlock();
                }
            }

            if (X1 && !X1Flag) {    //GROUND JUNCTION TOGGLE
                X1Flag = true;
                if (Servos.Arm.armState == "ActiveLow" || Servos.Arm.armState == "INIT") {
                    Servos.Arm.goActivePick();
                    Servos.Rotate.rotatePick();
                    sleep(200);
                    Servos.Arm.goPickTele();
                } else if (Servos.Arm.armState == "PICKTELE") {
                    Servos.Arm.goActiveStable();
                    Servos.Arm.goInit();
                    Servos.Rotate.rotatePick();
                }
            }

            // Elevator extensions

            if (A1) {                 //ELEVATOR LOW

                elevatortarget=LOW;
            }

            if (Y1) {                 //ELEVATOR MID

                elevatortarget=MID;
            }

            if (UP1) {                //ELEVATOR HIGH
                elevatortarget=HIGH;
            }

            if (DOWN1) {              //ELEVATOR HOME
                elevatortarget=HOME;
                Servos.Gripper.Unlock();
            }

            if (LEFT1) {              //CONE GRIPPING POSITION WITHOUT SLIDERS
                Servos.Gripper.openGripper();
                Servos.Arm.goActivePick();
                Servos.Rotate.rotatePick();
                sleep(200);
                Servos.Arm.goPickTele();
            }

            if (RIGHT1) {             //GOTO LOW POLE
                Servos.Gripper.closeGripper();
                sleep(200);
                Servos.Arm.goInit();
                Servos.Arm.goActiveLow();
                slidertarget=HOME;
            }


            // High junction cycle

            if (START1 && !START1Flag) {             // TRANSFER CYCLE
                START1Flag = true;
                if (Servos.Arm.armState == "INIT" && SlideFinalPos > 60) {
                    Servos.Gripper.Unlock();
                    Servos.Arm.goActivePick();
                    Servos.Rotate.rotatePick();
                    Servos.Gripper.openGripper();
                    sleep(300);
                    Servos.Arm.goPickTele();
                    sleep(300);
                    slidertarget=TEST;
                }
                else if (Servos.Arm.armState == "PICKTELE"){
                    drive.followTrajectorySequenceAsync(cycle);
                }

            }
            // // FLAGS FOR TOGGLE COMMANDS\
            if (!B1) {                //CONE TOGGLE FLAG
                B1Flag = false;
            }

            if (!X1) {                //GROUND JUNCTION TOGGLE FLAG
                X1Flag = false;
            }

            if (RTG1 <= 0.7) {        //GRIPPER TOGGLE FLAG
                RTG1Flag = false;
            }

            if (!RB1) {               //CONE TRANSFER FLAG
                RB1Flag = 0;
            }

            if (!START1){
                START1Flag = false;
            }

            telemetry.addData("ElevateFinalPos", ElevateFinalPos);
            telemetry.addData("Elevatetarget", elevatortarget);
            telemetry.addData("SliderFinalPos", SlideFinalPos);
            telemetry.addData("Slidetarget", slidertarget);
            telemetry.addData("Current ElevateRight", ElevateRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Current ElevateLeft", ElevateLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Current SlideRight", SlideRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Current SlideLeft", SlideLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("LeftFront", drive.leftFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("RightFront", drive.rightFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("LeftRear", drive.leftRear.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("RightRear", drive.rightRear.getCurrent(CurrentUnit.AMPS));
            telemetry.update();

        }

    }
}