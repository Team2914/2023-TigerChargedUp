package com.team2914.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.team2914.robot.Constants.AutoConstants;
import com.team2914.robot.subsystems.Drivetrain;
import com.team2914.robot.subsystems.Vision;
import com.team2914.robot.utils.MiscUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CommandAlignScore extends CommandBase {
    private final Drivetrain drivetrain;

    public CommandAlignScore(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {

    }

    @Override 
    public void execute() {
        Vision photon = Vision.getInstance();
        PhotonTrackedTarget target = photon.getBestTarget();
        if (target == null) return;

        PIDController rotController = MiscUtil.pidControllerFromConstants(AutoConstants.PID_ROT_CONSTANTS);
        int id = target.getFiducialId();
        double targetAngle = (id == 1 || id == 2 || id == 3) ? Math.PI : 0;
        double rotationSpeed = -rotController.calculate(target.getYaw(), targetAngle);
        drivetrain.drive(0, 0, rotationSpeed);
    }
}
