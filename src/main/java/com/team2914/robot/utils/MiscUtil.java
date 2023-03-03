package com.team2914.robot.utils;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.PIDController;

public class MiscUtil {
    public static List<Integer[]> bresenham(int x0, int y0, int x1, int y1) {
        List<Integer[]> out = new ArrayList<>();
        int dx = Math.abs(x1 - x0);
        int sx = (x0 < x1) ? 1 : -1;
        int dy = -Math.abs(y1 - y0);
        int sy = (y0 < y1) ? 1 : -1;
        int error = dx + dy;

        while (true) {
            out.add(new Integer[] { x0, y0 });

            if (x0 == x1 && y0 == y1) break;

            int e2 = 2 * error;
            if (e2 >= dy) {
                if (x0 == x1) break;

                error += dy;
                x0 += sx;
            }
            if (e2 <= dx) {
                if (y0 == y1) break;

                error += dx;
                y0 += sy;
            }
        }

        return out;
    }

    // Taken from BaseAutoBuilder - PathPlannerLib
    public static PIDController pidControllerFromConstants(PIDConstants constants) {
        return new PIDController(constants.kP, constants.kI, constants.kD, constants.period);
    }
}
