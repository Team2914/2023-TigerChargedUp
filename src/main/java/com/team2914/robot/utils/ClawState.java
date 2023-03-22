// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2914.robot.utils;

/** Add your docs here. */
public class ClawState {
    public static boolean hasGamePiece = false;
    public static String gamePieceType = null;
    public static boolean closed = false;
    public static int intaking = 0;
    // 0: ground; 1: lifted; 2: med; 3: high
    public static int liftLevel = 0;

    public static boolean hasCone() {
        return gamePieceType == "cone";
    }

    public static boolean hasCube() {
        return gamePieceType == "cube";
    }

    public static void setCone() {
        gamePieceType = "cone";
    }

    public static void setCube() {
        gamePieceType = "cube";
    }

    public static void setIntaking() {
        intaking = 1;
    }

    public static void setOuttaking() {
        intaking = -1;
    }

    public static void setStopIntake() {
        intaking = 0;
    }

    public static boolean getIntaking() {
        return intaking == 1;
    }

    public static boolean getOuttaking() {
        return intaking == -1;
    }

    public static boolean getIntakeStopped() {
        return intaking == 0;
    }
}
