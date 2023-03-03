package com.team2914.robot.utils;

public class MathUtil {
    public static double encodersToDegrees(double ticks, double gearRatio, int ticksPerRev) {
        return ticks * (360.0 / (gearRatio * ticksPerRev));
    }

    public static int degreesToEncoders(double deg, double gearRatio, int ticksPerRev) {
        return (int)(deg / (360.0 / (gearRatio * ticksPerRev)));
    }
}
