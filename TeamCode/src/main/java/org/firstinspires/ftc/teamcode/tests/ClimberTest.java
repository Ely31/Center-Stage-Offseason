package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Climber;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.SteeringArm;

@TeleOp(name="",group="test")
public class ClimberTest extends LinearOpMode {
    // Pre-init
    Lift lift;
    SteeringArm arm;
    Climber climber;

    enum ClimbingState{
        REDUCE_SLACK,
        HOLD,
        CLIMB
    }

    ClimbingState climbingState = ClimbingState.REDUCE_SLACK;
    boolean isClimbing = false;
    boolean prevClimbingInput = false;
    double climberSlackPullTime = 1.3;
    @Override
    public void runOpMode() {
        // Init
        lift = new Lift(hardwareMap);
        arm = new SteeringArm(hardwareMap);
        climber = new Climber(hardwareMap);
        ElapsedTime climberTimer = new ElapsedTime();


        waitForStart();
        // Pre-run
        while (opModeIsActive()) {
            // TeleOp loop

            //arm.setBothGrippersState(gamepad1.x);

            if ((gamepad2.left_bumper && gamepad2.right_bumper) && !prevClimbingInput){
                isClimbing = !isClimbing;
                // Do this to prevent crashing by running to a pos before setting one
                climber.setClimberTargetPos(climber.getClimberPos());
                climberTimer.reset();
            }
            prevClimbingInput = gamepad2.left_bumper && gamepad2.right_bumper;

            if(isClimbing){
                switch(climbingState){
                    case REDUCE_SLACK:
                        arm.setPivotPos(0.1);
                        climber.setClimberPower(-1);
                        lift.setLiftHeight(Climber.targetLiftHeight);
                        lift.update();

                        if (climberTimer.seconds() > climberSlackPullTime) {
                            climber.setClimberPower(0);
                            climber.setClimberTargetPos(climber.getClimberPos());
                            climbingState = climbingState.HOLD;
                        }
                        break;

                    case HOLD:
                        climber.climberGoToTargetPos();
                        lift.update();
                        if(!(gamepad2.left_stick_y == 0)){
                            climbingState = climbingState.CLIMB;
                        }
                        break;

                    case CLIMB:
                        climber.setClimberPower(-gamepad2.left_stick_y);
                        // Lift things
                        // Let it coast and be pulled up if
                        lift.setRawLiftPowerDangerous(0);
                        resetLiftController();
                        // Update so we can get the lift's position
                        lift.update(false);

                        if(gamepad2.left_stick_y == 0){
                            Climber.targetLiftHeight = lift.getLiftHeight();
                            lift.setLiftHeight(Climber.targetLiftHeight);
                            climber.setClimberTargetPos(climber.getClimberPos());
                            climbingState = climbingState.HOLD;
                        }
                        break;
                }
            }
            else{
                lift.retractLift();
                arm.pivotGoToIntake();
            }

            telemetry.addData("climbing state", climbingState.name());
            lift.disalayDebug(telemetry);
            climber.disalayDebug(telemetry);
            arm.displayDebug(telemetry);
            telemetry.update();
        }
    }

    void resetLiftController(){
        lift.setCoefficients(Lift.liftCoeffs);
    }
}
