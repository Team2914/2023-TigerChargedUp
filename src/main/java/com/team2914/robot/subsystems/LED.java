package com.team2914.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private static LED instance = null;
    private final Spark blinkin;

    private LED() {
        blinkin = new Spark(0);
        blinkin.set(0.57);
    }

    public static LED getInstance() {
        if (instance == null) {
            instance = new LED();
        }

        return instance;
    }

    @Override
    public void periodic() {
        blinkin.set(0.77);
    }
}
