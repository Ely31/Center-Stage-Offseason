package org.firstinspires.ftc.teamcode.util;

public class PIDCoefficients {
    public double kP, kI, kD;

    public PIDCoefficients(double p, double i, double d){
        kP = p;
        kI = i;
        kD = d;
    }
}
