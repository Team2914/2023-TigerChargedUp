// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2914.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team2914.robot.subsystems.Claw;
import com.team2914.robot.subsystems.Lift;
import com.team2914.robot.utils.ClawState;

public class IntakeGamePiece extends CommandBase {
  private final Claw claw;
  private final Lift lift;

  public IntakeGamePiece(Claw claw, Lift lift) {
    this.claw = claw;
    this.lift = lift;
    
    addRequirements(claw, lift);
  }

  @Override
  public void initialize() {
    ClawState.hasGamePiece = false;
    //claw.openClaw();
  }

  @Override
  public void execute() {
    if (!ClawState.hasGamePiece && !ClawState.closed) {
      claw.runIntake();
    } else if (ClawState.hasGamePiece && !ClawState.closed) {
      claw.stopIntake();
      claw.closeClaw();
    } else if (ClawState.hasGamePiece && ClawState.closed && ClawState.liftLevel == 0) {
      claw.closeClaw();
      lift.liftGamePiece();
    } else if (ClawState.hasGamePiece && ClawState.closed && ClawState.liftLevel == 3) {
      claw.closeClaw();
    }
  }

  @Override
  public boolean isFinished() {
    return ClawState.liftLevel > 0;
  }
}
