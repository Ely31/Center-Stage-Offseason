package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Climber {

    DcMotor climberMotor;
    public static double targetLiftHeight = 16.25;
    public static double hangingHeight = 16.25;

    public static int slackPullPos = -3300;
    public static int closeEnoughRange = 100;

    enum ClimbingState {
        REDUCE_SLACK,
        HOLD,
        CLIMB
    }
    ClimbingState climbingState = ClimbingState.REDUCE_SLACK;

    public Climber(HardwareMap hwmap){
        climberMotor = hwmap.get(DcMotor.class, "climber");
        climberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        zero();
    }

    public void update(){
        switch (climbingState) {
            case REDUCE_SLACK:
                // Pull in slack
                setTargetPos(slackPullPos);
                goToTargetPos();

                lift.setHeight(targetLiftHeight);
                lift.update();

                if ((getPos() - slackPullPos) < closeEnoughRange) {
                    setPower(0);
                    setTargetPos(getPos());
                    climbingState = ClimbingState.HOLD;
                }
                break;

            case HOLD:
                goToTargetPos();
                lift.update();

                if (!(gamepad2.left_stick_y == 0)) {
                    climbingState = ClimbingState.CLIMB;
                }
                break;

            case CLIMB:
                setPower(-gamepad2.left_stick_y);
                // Lift things
                // Let it coast and be pulled up if
                lift.setRawPowerDangerous(0);
                resetLiftController();
                // Update so we can get the lift's position
                lift.update(false);

                if (gamepad2.left_stick_y == 0) {
                    targetLiftHeight = lift.getHeight();
                    lift.setHeight(targetLiftHeight);
                    setTargetPos(getPos());
                    climbingState = ClimbingState.HOLD;
                }
                break;
        }
    }

    public void zero(){
        climberMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setPower(double power){
        climberMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climberMotor.setPower(power);
    }
    public void setTargetPos(int pos){
        climberMotor.setTargetPosition(pos);
    }
    public void goToTargetPos(){
        climberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climberMotor.setPower(1);
    }
    public int getPos(){
        return climberMotor.getCurrentPosition();
    }

    public void disalayDebug(Telemetry telemetry){
        telemetry.addLine("CLIMBER");
        telemetry.addData("Power", climberMotor.getPower());
        telemetry.addData("Target pos", climberMotor.getTargetPosition());
        telemetry.addData("Lift target height", targetLiftHeight);
    }
}
