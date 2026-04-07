// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RightFlywheel;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/** SetFlywheelVelocity command for the right flywheel. */
public class RightSetFlywheelVelocity extends Command {
    private DoubleSupplier m_setpoint;
    private double m_tolerance;
    private RightFlywheel m_rightFlywheel;

    /**
     * A command to set the closed loop setpoint of the right flywheel, in rotations per second.
     *
     * @param setpoint  The desired speed of the rightFlywheel (rot/s)
     * @param tolerance The tolerance for error (rot/s)
     * @param rightFlywheel  The rightFlywheel to use
     */
    public RightSetFlywheelVelocity(DoubleSupplier setpoint, double tolerance, RightFlywheel rightFlywheel) {
        m_setpoint = setpoint;
        m_tolerance = tolerance;
        m_rightFlywheel = rightFlywheel;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_rightFlywheel);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_rightFlywheel.updateSetpoint(m_setpoint.getAsDouble());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Math.abs(m_setpoint.getAsDouble() - m_rightFlywheel.getSetpoint()) > 0.01) {
            m_rightFlywheel.updateSetpoint(m_setpoint.getAsDouble());
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
            return Math.abs(m_rightFlywheel.getVelocity() - m_setpoint.getAsDouble()) < m_tolerance;
        }
    }
}
