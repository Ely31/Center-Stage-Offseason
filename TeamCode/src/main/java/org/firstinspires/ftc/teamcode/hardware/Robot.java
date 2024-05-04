package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.AutoToTele;
import org.firstinspires.ftc.teamcode.util.TimeUtil;
import org.firstinspires.ftc.teamcode.util.Utility;
import org.firstinspires.ftc.teamcode.vision.workspace.PropDetector;

public class Robot {
    MecanumDrive drive;
    Intake intake;
    Lift lift;
    SteeringArm arm;
    
    DroneLauncher droneLauncher;
    Camera camera;
    PropDetector pipeline;
    TimeUtil timeUtil;

    ElapsedTime pivotTimer = new ElapsedTime();
    ElapsedTime gripperTimer = new ElapsedTime();
    ElapsedTime doubleTapTimer = new ElapsedTime();

    boolean isAuto;

    boolean hadAnyPixelsWhenPremoved = false;
    boolean poking;
    boolean prevProgressInput;
    boolean prevPokingInput;

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
    ScoringState scoringState = ScoringState.INTAKING;
    
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
        }
    }
    public void resetTimers(){
        pivotTimer.reset();
        gripperTimer.reset();
        doubleTapTimer.reset();
    }
    
    public void updateScoringMech(boolean progressInput, boolean backInput, boolean dropOneInput, boolean dropBothInput, boolean togglePokerInput, boolean bumpUp){
        switch (scoringState){
            case INTAKING:
                // Prevent arm hitting stuff near the intake because we swapped it for a speed
                if (Utility.withinErrorOfValue(lift.getLiftHeight(), 0, 2)){
                    arm.pivotGoToIntake();
                } else {
                    arm.preMove();
                }
                // Make sure the deposit doesn't crash into stuff
                arm.centerSteer();
                // Wait to retract the lift until the arm is safely away from the board
                if (pivotTimer.seconds() > SteeringArm.pivotAwayFromBordTime) {
                    lift.retractLift();
                }
                // Put up the stopper so pixels don't fly out the back, but only after the arm's back to avoid hooking a pixel on it
                if (pivotTimer.seconds() > 5) {
                    arm.setStopperState(true);
                }
                // If we had pixels when premoved and now moved back down to intaking,
                // hold onto them until the arm gets all the way down so they don't fly out.
                // Open them to intake once the arm gets all the way there.
                arm.setBothGrippersState(!arm.armIsDown());
                // Reset poker
                poking = false;
                // Switch states when bumper pressed
                // Or, (and this'll happen 95% of the time) when it has both pixels
                if ((arm.pixelIsInBottom() && arm.pixelIsInTop()) || (!prevProgressInput && progressInput)){
                    // Grab 'em and move the arm up
                    arm.setBothGrippersState(true);
                    gripperTimer.reset();
                    scoringState = ScoringState.GRIPPING;
                }
                break;

            case GRIPPING:
                if (gripperTimer.seconds() > SteeringArm.gripperActuationTime){
                    scoringState = ScoringState.PREMOVED;
                    doubleTapTimer.reset();
                }
                break;

            case PREMOVED:
                arm.preMove();
                arm.centerSteer();
                // Wait to retract the lift until the arm is safely away from the board
                if (pivotTimer.seconds() > SteeringArm.pivotAwayFromBordTime) lift.retractLift();
                // Just make sure we're still holding on
                arm.setBothGrippersState(true);
                arm.setStopperState(false);
                // Toggle the intake off to prevent sucking in pixels when the arm isn't there
                intake.forceToggleOff();
                // Spit out just a little to avoid dragging a third under the tubing
                // Using the gripper timer for this is hacky but oh well
                if (gripperTimer.seconds() < (SteeringArm.gripperActuationTime + 0.2)) intake.reverse();
                else intake.off();
                // Lift up the intake so it's less likely to get crunched
                intake.goToVertical();

                // Reset poker
                poking = false;

                // Switch states when bumper pressed
                // Don't go to scoring if the arm has just been premoved though, this happens when the automatic raises it but the driver
                // tries to manually raise it at the same time. This timer prevents that.
                if (!prevProgressInput && progressInput && doubleTapTimer.seconds() > 4.5){
                    scoringState = ScoringState.SCORING;
                    // Save this info to prevent it from going down right away if you have nothing
                    hadAnyPixelsWhenPremoved = (arm.pixelIsInBottom() || arm.pixelIsInTop());
                    pivotTimer.reset();
                }
                // Go back to intaking if the arm pulled up before getting both pixels
                if (backInput){
                    scoringState = ScoringState.INTAKING;
                }
                break;

            case SCORING:
                arm.pivotScore();
                lift.extendLift();
                if (dropOneInput) {
                    arm.setBottomGripperState(false);
                    poking = false;
                }
                if (dropBothInput) {
                    arm.setBothGrippersState(false);
                    poking = false;
                }
                // Toggle the poker
                if (togglePokerInput && !prevPokingInput){
                    poking = !poking;
                }
                prevPokingInput = togglePokerInput;
                arm.setStopperState(poking);
                // Wait for the arm to move a bit so the deposit doesn't hit the bot
                if (pivotTimer.seconds() > 0.25) arm.updateSteer(hasCalibratedFC ? drive.getHeading() : getCorrectedSteeringHeading());
                // Switch states when the bumper is pressed or both pixels are gone if autoRetract is on
                if (!arm.getTopGripperState() && !arm.getBottomGripperState()){
                    scoringState = ScoringState.BUMPING;
                }
                // Go back to premoved if we wish
                if (backInput){
                    scoringState = ScoringState.PREMOVED;
                    pivotTimer.reset();
                }
                break;

            case BUMPING:
                arm.updateSensors(true, false);
                // Raise the lift up gradually to get clear of pixels
                lift.setExtendedPos(lift.getExtendedPos() + 0.15);
                lift.extendLift();
                // Add bumper to escape as well because we've had bugs with it staying up forever
                if ((!arm.pixelIsInTop() && !arm.pixelIsInBottom()) || progressInput){
                    scoringState = ScoringState.BUMPING_UP;
                    // Bump up
                    lift.setExtendedPos(lift.getExtendedPos() + 2);
                }
                // Once it's gone up enough, switch states and retract
                // Add bumper to escape as well because we've had bugs with it staying up forever
                if (Utility.withinErrorOfValue(lift.getLiftHeight(), lift.getExtendedPos(), 0.5) || gamepad2.right_bumper) {
                    // Reset that back to normal because we temporarily changed it
                    lift.setExtendedPos(lift.getExtendedPos() - 2);
                    scoringState = ScoringState.INTAKING;
                    // Reset timer so the clock ticks on the arm being away from the board
                    pivotTimer.reset();
                }
                break;
        }
        prevProgressInput = progressInput;
    }


    public void write(){

    }

    public void read(){
        if (scoringState == ScoringState.INTAKING) arm.updateSensors(true, true);
        if (scoringState == ScoringState.BUMPING) arm.updateSensors(true, false);
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
        }
    }
}
