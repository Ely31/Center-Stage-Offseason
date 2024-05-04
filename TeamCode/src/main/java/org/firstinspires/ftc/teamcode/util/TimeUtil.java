package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class TimeUtil {
    private final int endgameTime = 120;
    private ElapsedTime timer = new ElapsedTime();

    double latestLoopTime = 0;
    private double lastTime = 0;
    public ArrayList<Double> lastLoopTimes = new ArrayList<>();
    private double averageLoopTime = 0;

    public enum Period
    {
        TELEOP,
        ENDGAME
    }
    // Make a current and last period for rising edge detectors
    private volatile  Period currentPeriod;
    private volatile  Period lastPeriod;

    private boolean justEnteredEndgame = false;

    public Period getPeriod(){
        return currentPeriod;
    }
    public boolean isInEndgame(){
        return (currentPeriod == Period.ENDGAME);
    }
    public boolean isInTeleop(){
        return currentPeriod == Period.TELEOP;
    }

    public boolean justEnteredEndgame(){
        return justEnteredEndgame;
    }

    public double getAverageLoopTime(){
        return averageLoopTime;
    }
    public double getLatestLoopTime(){
        return latestLoopTime;
    }

    public void resetTimer(){
        timer.reset();
    }

    public void update(double currentTime){
        // Set which game period we're in by checking the time
        if ((currentTime/1000) < endgameTime) currentPeriod = Period.TELEOP;
        if ((currentTime/1000) > endgameTime) currentPeriod = Period.ENDGAME;

        // Get how long the loop took
        latestLoopTime = currentTime - lastTime;
        lastTime = currentTime;

        // If the array is full, make room for the new time
        if (lastLoopTimes.size() >= 50){
            lastLoopTimes.remove(0);
        }
        // Add the latest time to the array
        lastLoopTimes.add(latestLoopTime);

        // Calculate the average of all entries in the array
        double totalOfTimes = 0;
        for (int i = 0; i< lastLoopTimes.size(); i++){
            totalOfTimes += lastLoopTimes.get(i);
        }
        averageLoopTime = totalOfTimes / lastLoopTimes.size();

        // Rising edge detectors
        justEnteredEndgame = currentPeriod == Period.ENDGAME && lastPeriod == Period.TELEOP;

        lastPeriod = currentPeriod;
    }
    public void update(){
        update(timer.milliseconds());
    }

    public void displayDebug(Telemetry telemetry){
        telemetry.addLine("TIME");
        telemetry.addData("avg loop time (ms)", getAverageLoopTime());
        telemetry.addData("period", getPeriod());
        telemetry.addData("time", timer.seconds());
    }
}
