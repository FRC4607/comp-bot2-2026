// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RightTurret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/** MoveTurretToPosition command for the right turret. */
public class RightMoveTurretToPosition extends Command {
    private DoubleSupplier m_degrees;
    private double m_tolerance;
    private RightTurret m_rightTurret;

    /**
     * A command that sets the setpoint of the right turret, in degrees.
     *
     * @param degrees  The position to drive towards, in degrees.
     * @param tolerance The tolerance for error, in degrees. If set to 0, the command will not end.
     * @param rightTurret    The rightTurret to use
     */
    public RightMoveTurretToPosition(DoubleSupplier degrees, double tolerance, RightTurret rightTurret) {
        m_degrees = degrees;
        m_tolerance = tolerance;
        m_rightTurret = rightTurret;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_rightTurret);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_rightTurret.updateSetpoint(m_degrees.getAsDouble());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Update the setpoint if the target has changed by more than 0.01 degrees
        if (Math.abs(m_degrees.getAsDouble() - m_rightTurret.getSetpoint()) > 0.01) {
            m_rightTurret.updateSetpoint(m_degrees.getAsDouble());
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_tolerance == 0) {
            return false;
        } else {
            return Math.abs(m_rightTurret.getPosition() - m_degrees.getAsDouble()) < m_tolerance;
        }
    }
}
