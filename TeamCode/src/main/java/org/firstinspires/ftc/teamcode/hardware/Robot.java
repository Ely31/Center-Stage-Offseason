package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.AutoToTele;
import org.firstinspires.ftc.teamcode.util.TimeUtil;
import org.firstinspires.ftc.teamcode.vision.workspace.PropDetector;

public class Robot {
    MecanumDrive drive;
    Intake intake;
    Lift lift;
    SteeringArm arm;
    
    DroneLauncher droneLauncher;
    Climber climber;
    Camera camera;
    PropDetector pipeline;
    TimeUtil timeUtil;

    boolean isAuto;

    enum Mode {
        MAIN,
        CLIMBING,
        LIFT_ADJUST
    }
    Mode mode = Mode.MAIN;

    enum ScoringState {
        INTAKING,
        GRIPPING,
        PREMOVED,
        SCORING,
        BUMPING
    }
    ScoringState scoringstate = ScoringState.INTAKING;
    
    public Robot (HardwareMap hwmap, boolean isAuto){
        drive = new MecanumDrive(hwmap, AutoToTele.endOfAutoPose);
        intake = new Intake(hwmap);
        lift = new Lift(hwmap);
        arm = new SteeringArm(hwmap);
        timeUtil = new TimeUtil();

        this.isAuto = isAuto;
        if (isAuto){
            camera = new Camera(hwmap, pipeline);
        } else {
            droneLauncher = new DroneLauncher(hwmap);
            climber = new Climber(hwmap);
        }
    }



    public void write(){

    }

    public void read(){
        if (scoringstate == ScoringState.INTAKING) arm.updateSensors(true, true);
        if (scoringstate == ScoringState.BUMPING) arm.updateSensors(true, false);
    }
    
    public void displayDebug(Telemetry t){
        t.addLine("SUBSYSTEMS");
        t.addLine();
        //drive.displayDebug(t);
        intake.displayDebug(t);
        lift.disalayDebug(t);
        arm.displayDebug(t);
        timeUtil.displayDebug(t);

        if (!isAuto){
            droneLauncher.displayDebug(t);
            climber.disalayDebug(t);
        }
    }
}
