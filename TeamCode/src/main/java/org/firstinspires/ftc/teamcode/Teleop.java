package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;

import java.util.List;

@TeleOp
public class Teleop extends LinearOpMode {
    Robot bot;

    @Override
    public void runOpMode() throws InterruptedException {
        bot = new Robot(hardwareMap, false);
        // Manual bulk reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        waitForStart();
        bot.resetTimers();
        // Begin TeleOp loop
        while(opModeIsActive() && !isStopRequested()){
            // Bulk reads
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            bot.driveFieldCentric(new PoseVelocity2d(new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y), gamepad1.right_stick_x));
            if (gamepad1.share) bot.calibrateFieldCentric();
            /*
            bot.controlIntake(gamepad1.left_stick_button, gamepad1.right_stick_button);
            bot.manipulateIntakeHeight(
                    gamepad1.dpad_up,
                    gamepad1.dpad_down,
                    gamepad1.a,
                    gamepad1.y,
                    false
            );

            bot.updateScoringMech(
                    gamepad1.right_bumper,
                    gamepad1.dpad_left,
                    gamepad1.left_trigger > 0.5,
                    gamepad1.left_bumper,
                    -gamepad1.right_stick_y,
                    gamepad1.b,
                    2
            );

            bot.setDronelauncherState(gamepad1.touchpad);


             */
            bot.displayDebug(telemetry);
            telemetry.update();
        }
    }
}
