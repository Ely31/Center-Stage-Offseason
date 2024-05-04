package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class DroneLauncher {
    Servo triggerServo;
    public static double releasePos = 0.8;
    public static double holdPos = 0.53;

    boolean holding = true;

    public DroneLauncher(HardwareMap hwmap){
        triggerServo = hwmap.get(Servo.class, "launcher");
        hold();
    }

    public void hold(){
        holding = true;
        triggerServo.setPosition(holdPos);
    }
    public void release(){
        holding = false;
        triggerServo.setPosition(releasePos);
    }

    public void displayDebug(Telemetry t){
        t.addLine("DRONE LAUNCHER");
        t.addData("Holding", holding);
        t.addData("Trigger pos", triggerServo.getPosition());
    }
}
