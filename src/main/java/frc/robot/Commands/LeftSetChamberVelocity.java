// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LeftChamber;
import frc.robot.subsystems.LeftFlywheel;
import frc.robot.subsystems.LeftHood;
import frc.robot.subsystems.LeftTurret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/** SetChamberVelocity command. */
public class LeftSetChamberVelocity extends Command {
    private double m_setpoint;
    private double m_tolerance;
    private boolean m_waitForTurret;

    private LeftChamber m_leftChamber;
    private LeftTurret m_leftTurret;
    private LeftHood m_leftHood;
    private LeftFlywheel m_leftFlywheel;


    /** Creates a new SetChamberVelocity. */
    public LeftSetChamberVelocity(double setpoint, double tolerance, boolean waitForTurret, LeftChamber leftChamber, LeftTurret leftTurret, LeftHood leftHood, LeftFlywheel leftFlywheel) {
        m_setpoint = setpoint;
        m_tolerance = tolerance;
        m_leftChamber = leftChamber;
        m_leftTurret = leftTurret;
        m_leftHood = leftHood;
        m_leftFlywheel = leftFlywheel;

        m_waitForTurret = waitForTurret;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_leftChamber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (!m_waitForTurret) {
            m_leftChamber.updateSetpoint(m_setpoint);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_waitForTurret) {
            if (((Math.abs(m_leftTurret.getPosition() - m_leftTurret.getSetpoint()) < 5)
                && (Math.abs(m_leftHood.getPosition() - m_leftHood.getSetpoint()) < 0.2)
                && (Math.abs(m_leftFlywheel.getVelocity() - m_leftFlywheel.getSetpoint()) < 10)
                && (Math.abs(m_leftChamber.getSetpoint() - m_setpoint) > 0.1))
                || m_leftChamber.isDisabled()) {
                m_leftChamber.updateSetpoint(m_setpoint);
            } else if ((Math.abs(m_leftChamber.getSetpoint() - m_setpoint) > 0.1)) {
                m_leftChamber.updateSetpoint(0);
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(m_leftChamber.getVelocity() - m_setpoint) < m_tolerance;
    }
}
