package com.pathplanner.lib;

import java.util.List;
import java.util.ArrayList;
import com.pathplanner.lib.PathPlannerTrajectory.Waypoint;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.geometry.Rotation2d;

public class PathPlannerEx extends PathPlanner {
    public static PathPlannerTrajectory tigerGeneratePath(PathConstraints constraints, boolean reversed, List<PathPoint> points) {
        if (points.size() < 2) {
            throw new IllegalArgumentException(
                "Error generating trajectory.  List of points in trajectory must have at least two points.");
        }
      
        List<Waypoint> waypoints = new ArrayList<>();
        for (int i = 0; i < points.size(); i++) {
            PathPoint p0 = (i > 0) ? points.get(i - 1) : null;
            PathPoint p1 = points.get(i);
            PathPoint p2 = (i < points.size() - 1) ? points.get(i + 1) : null;

            Translation2d prevControl = null;
            Translation2d nextControl = null;
            Rotation2d angle = p1.holonomicRotation;

            if (p2 != null) {
                angle = Rotation2d.fromRadians(
                    Math.atan2(p2.position.getY() - p1.position.getY(), p2.position.getX() - p1.position.getX()));
            }

            if (p0 != null) {
                prevControl = 
                    p1.position.minus(
                        new Translation2d(
                            1.5 * Math.cos(angle.getRadians()), 
                            1.5 * Math.sin(angle.getRadians())
                    )
                );
            }
            
            nextControl = 
                p1.position.plus(
                    new Translation2d(
                        1.5 * Math.cos(angle.getRadians()),
                        1.5 * Math.sin(angle.getRadians())
                    )
                );

            waypoints.add(
                new Waypoint(
                    p1.position, 
                    prevControl, 
                    nextControl, 
                    p1.velocityOverride, 
                    p1.holonomicRotation, 
                    false, 
                    false, 
                    new PathPlannerTrajectory.StopEvent()
                )
            );
        }

        return new PathPlannerTrajectory(waypoints, new ArrayList<>(), constraints, reversed, false);
    }
}
