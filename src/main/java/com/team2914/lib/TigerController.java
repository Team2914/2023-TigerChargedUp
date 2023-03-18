package com.team2914.lib;

import edu.wpi.first.wpilibj.Joystick;

public class TigerController {
    protected final Joystick joystick;

    public TigerController(int port) {
        joystick = new Joystick(port);
    }

    public double getX() {
        return joystick.getX();
    }

    public double getY() {
        return joystick.getY();
    }

    public double getZ() {
        return joystick.getZ();
    }

    public double getSlider() {
        return joystick.getRawAxis(3);
    }

    public void configureButtons() {
        
    }

    public Joystick getJoystick() {
        return joystick;
    }
}
