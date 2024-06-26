package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Lift;

@TeleOp(name="",group="test")
public class LiftTest extends LinearOpMode {
    // Pre-init
    Lift lift;
    @Override
    public void runOpMode() {
        // Init
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        lift = new Lift(hardwareMap);
        lift.setCoefficients(Lift.liftCoeffs);
        waitForStart();
    
        // Pre-run
        lift.retractLift();
        while (opModeIsActive()) {
            // TeleOp loop
            if(gamepad1.dpad_left) lift.retractLift();
            else if (gamepad1.dpad_right) lift.extendLift();

            if (gamepad1.dpad_up) lift.editExtendedPos(0.1);
            if (gamepad1.dpad_down) lift.editExtendedPos(-0.1);

            lift.update();
            lift.disalayDebug(telemetry);
            telemetry.update();
        }
    }
}
