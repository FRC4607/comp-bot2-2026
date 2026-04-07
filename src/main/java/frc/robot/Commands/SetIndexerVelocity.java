// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/** SetIndexerVelocity command. */
public class SetIndexerVelocity extends Command {
    private double m_setpoint;
    private double m_tolerance;
    private Indexer m_indexer;

    /**
     * A command to set the closed loop setpoint of the leftFlywheel, in motor rotations per second.
     *
     * @param setpoint  The desired speed of the leftFlywheel (rot/s)
     * @param tolerance The tolerance for error (rot/s)
     * @param indexer  The indexer to use
     */
    public SetIndexerVelocity(double setpoint, double tolerance, Indexer indexer) {
        m_setpoint = setpoint;
        m_tolerance = tolerance;
        m_indexer = indexer;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_indexer);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_indexer.updateSetpoint(m_setpoint);
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
        return Math.abs(m_indexer.getVelocity() - m_setpoint) < m_tolerance;

    }
}
