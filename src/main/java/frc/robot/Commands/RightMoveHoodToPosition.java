// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RightHood;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/** MoveHoodToPosition command for the right hood. */
public class RightMoveHoodToPosition extends Command {
    private double m_setpoint;
    private double m_tolerance;
    private RightHood m_rightHood;

    // TODO: Convert to degrees once converted in the rightHood subsystem

    /**
     * A command to set the closed loop setpoint of the right hood, in motor rotations.
     *
     * @param setpoint  The position to drive to (motor rotations)
     * @param tolerance The tolerance for error (motor rotations)
     * @param rightHood      The rightHood to use.
     */
    public RightMoveHoodToPosition(double setpoint, double tolerance, RightHood rightHood) {
        m_setpoint = setpoint;
        m_tolerance = tolerance;
        m_rightHood = rightHood;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_rightHood);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_rightHood.updateSetpoint(m_setpoint);
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
        return Math.abs(m_rightHood.getPosition() - m_setpoint) < m_tolerance;
    }
}
