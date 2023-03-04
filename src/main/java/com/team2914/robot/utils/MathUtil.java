package com.team2914.robot.utils;

public class MathUtil {
    public static double encodersToRad(double ticks, double gearRatio, int ticksPerRev) {
        return ticks * (2 * Math.PI / (gearRatio * ticksPerRev));
    }

    public static int radToEncoders(double rad, double gearRatio, int ticksPerRev) {
        return (int)(rad / (2 * Math.PI / (gearRatio * ticksPerRev)));
    }
}
