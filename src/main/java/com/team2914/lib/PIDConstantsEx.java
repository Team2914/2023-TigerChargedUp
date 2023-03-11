package com.team2914.lib;

import com.pathplanner.lib.auto.PIDConstants;

public class PIDConstantsEx {
    public double kP;
    public double kI;
    public double kD;
    public double period;

    public PIDConstantsEx(double kP, double kI, double kD, double period) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.period = period;
    }

    public PIDConstantsEx(double kP, double kI, double kD) {
        this(kP, kI, kD, 0.02);
    }

    public String toString() {
        return "P: "+this.kP+" I: "+this.kI+" D: "+this.kD;
    }

}
