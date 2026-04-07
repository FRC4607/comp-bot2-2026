// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LeftFlywheel;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/** SetFlywheelVelocity command. */
public class LeftSetFlywheelVelocity extends Command {
    private DoubleSupplier m_setpoint;
    private double m_tolerance;
    private LeftFlywheel m_leftFlywheel;

    /**
     * A command to set the closed loop setpoint of the leftFlywheel, in rotations per second.
     *
     * @param setpoint  The desired speed of the leftFlywheel (rot/s)
     * @param tolerance The tolerance for error (rot/s)
     * @param leftFlywheel  The leftFlywheel to use
     */
    public LeftSetFlywheelVelocity(DoubleSupplier setpoint, double tolerance, LeftFlywheel leftFlywheel) {
        m_setpoint = setpoint;
        m_tolerance = tolerance;
        m_leftFlywheel = leftFlywheel;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_leftFlywheel);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_leftFlywheel.updateSetpoint(m_setpoint.getAsDouble());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Math.abs(m_setpoint.getAsDouble() - m_leftFlywheel.getSetpoint()) > 0.01) {
            m_leftFlywheel.updateSetpoint(m_setpoint.getAsDouble());
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
            return Math.abs(m_leftFlywheel.getVelocity() - m_setpoint.getAsDouble()) < m_tolerance;
        }
    }
}
