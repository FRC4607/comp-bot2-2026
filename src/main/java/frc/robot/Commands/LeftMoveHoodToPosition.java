// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LeftHood;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/** MoveHoodToPosition command. */
public class LeftMoveHoodToPosition extends Command {
    private double m_setpoint;
    private double m_tolerance;
    private LeftHood m_leftHood;

    // TODO: Convert to degrees once converted in the leftHood subsystem

    /**
     * A command to set the closed loop setpoint of the leftHood, in motor rotations.
     *
     * @param setpoint  The position to drive to (motor rotations)
     * @param tolerance The tolerance for error (motor rotations)
     * @param leftHood      The leftHood to use.
     */
    public LeftMoveHoodToPosition(double setpoint, double tolerance, LeftHood leftHood) {
        m_setpoint = setpoint;
        m_tolerance = tolerance;
        m_leftHood = leftHood;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_leftHood);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_leftHood.updateSetpoint(m_setpoint);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(m_leftHood.getPosition() - m_setpoint) < m_tolerance;
    }
}
