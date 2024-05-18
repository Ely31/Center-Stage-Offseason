package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PIDCoefficients;
import org.firstinspires.ftc.teamcode.util.PIDFController;
import org.firstinspires.ftc.teamcode.util.Utility;

@Config
public class Lift {
    DcMotorEx liftMotor;
    DcMotor climberMotor;
    public static double targetLiftHeightWhileClimbing = 16.25;
    public static double climberHangingHeight = 16.25;
    public static int climberSlackPullPos = -3300;
    public static int climberCloseEnoughRange = 100;

    public static double maxHeight = 26;
    public static double minHeight = 0;
    public static double extendedPos = 7;

    final double TICKS_PER_INCH = 537.7 / 5.93;

    public static PIDCoefficients liftCoeffs = new PIDCoefficients(0.5,0.15,0.04);
    PIDFController liftController = new PIDFController(liftCoeffs);

    enum ClimbingState {
        REDUCE_SLACK,
        HOLD,
        CLIMB
    }
    ClimbingState climbingState = ClimbingState.REDUCE_SLACK;

    public Lift(HardwareMap hwmap){
        liftMotor = hwmap.get(DcMotorEx.class, "lift");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        zeroLift();
        extendedPos = 7;
        setCoefficients(liftCoeffs);

        climberMotor = hwmap.get(DcMotor.class, "climber");
        climberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climberMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void updateClimbing(double pullInput){
        switch (climbingState) {
            case REDUCE_SLACK:
                // Pull in slack
                setClimberTargetPos(climberSlackPullPos);
                climberGoToTargetPos();

                setLiftHeight(targetLiftHeightWhileClimbing);
                update(true);

                if ((getClimberPos() - climberSlackPullPos) < climberCloseEnoughRange) {
                    setClimberPower(0);
                    setClimberTargetPos(getClimberPos());
                    climbingState = ClimbingState.HOLD;
                }
                break;

            case HOLD:
                climberGoToTargetPos();
                update(true);

                if (!(pullInput == 0)) {
                    climbingState = ClimbingState.CLIMB;
                }
                break;

            case CLIMB:
                setClimberPower(-pullInput);
                // Lift things
                // Let it coast and be pulled up if
                setRawLiftPowerDangerous(0);
                liftController.reset();
                // Update so we can get the lift's position
                update(false);

                if (pullInput == 0) {
                    targetLiftHeightWhileClimbing = getLiftHeight();
                    setLiftHeight(targetLiftHeightWhileClimbing);
                    setClimberTargetPos(getClimberPos());
                    climbingState = ClimbingState.HOLD;
                }
                break;
        }
    }

    public void zeroLift(){
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setCoefficients(PIDCoefficients coeffs){
        liftController = new PIDFController(coeffs);
        Lift.liftCoeffs = coeffs;
    }

    // Methods
    public void setLiftHeight(double height){
        liftController.setTargetPosition(Utility.clipValue(minHeight, maxHeight, height));
    }
    public double getLiftHeight(){
        return liftMotor.getCurrentPosition()/TICKS_PER_INCH;
    }
    public void retractLift(){
        setLiftHeight(0);}
    public void extendLift(){
        setLiftHeight(extendedPos);}

    public void editExtendedPos(double step){
        extendedPos += step;
        extendedPos = Utility.clipValue(minHeight, maxHeight, extendedPos);
    }
    public void setExtendedPos(double height){
        height = Utility.clipValue(minHeight, maxHeight, height);
        extendedPos = height;
    }
    public double getExtendedPos(){
        return extendedPos;
    }

    public void update(boolean updateLiftPID){
        // Hacky fix to make some of the climbing automation work
        if (updateLiftPID) {
            liftController.update(getLiftHeight());
        }
    }
    public void update(){
        this.update(true);
    }

    // DANGEROUS!
    public void setRawLiftPowerDangerous(double power){
        liftMotor.setPower(power);
    }

    public void setClimberPower(double power){
        climberMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climberMotor.setPower(power);
    }
    public void setClimberTargetPos(int pos){
        climberMotor.setTargetPosition(pos);
    }
    public void climberGoToTargetPos(){
        climberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climberMotor.setPower(1);
    }
    public int getClimberPos(){
        return climberMotor.getCurrentPosition();
    }

    public void disalayDebug(Telemetry telemetry){
        telemetry.addLine("LIFT");
        telemetry.addData("Current distance", "%.3f", getLiftHeight());
        telemetry.addData("Target distance", "%.3f", liftController.targetPosition);
        telemetry.addData("distance error","%.3f", liftController.lastError);
        telemetry.addLine("CLIMBER");
        telemetry.addData("Power", climberMotor.getPower());
        telemetry.addData("Target pos", climberMotor.getTargetPosition());
        telemetry.addData("Current pos", climberMotor.getCurrentPosition());
    }
}
