// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RightChamber;
import frc.robot.subsystems.RightFlywheel;
import frc.robot.subsystems.RightHood;
import frc.robot.subsystems.RightTurret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/** SetChamberVelocity command for the right chamber. */
public class RightSetChamberVelocity extends Command {
    private double m_setpoint;
    private double m_tolerance;
    private boolean m_waitForTurret;

    private RightChamber m_rightChamber;
    private RightTurret m_rightTurret;
    private RightHood m_rightHood;
    private RightFlywheel m_rightFlywheel;


    /** Creates a new RightSetChamberVelocity. */
    public RightSetChamberVelocity(double setpoint, double tolerance, boolean waitForTurret, RightChamber rightChamber, RightTurret rightTurret, RightHood rightHood, RightFlywheel rightFlywheel) {
        m_setpoint = setpoint;
        m_tolerance = tolerance;
        m_rightChamber = rightChamber;
        m_rightTurret = rightTurret;
        m_rightHood = rightHood;
        m_rightFlywheel = rightFlywheel;

        m_waitForTurret = waitForTurret;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_rightChamber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (!m_waitForTurret) {
            m_rightChamber.updateSetpoint(m_setpoint);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_waitForTurret) {
            if (((Math.abs(m_rightTurret.getPosition() - m_rightTurret.getSetpoint()) < 5)
                && (Math.abs(m_rightHood.getPosition() - m_rightHood.getSetpoint()) < 0.2)
                && (Math.abs(m_rightFlywheel.getVelocity() - m_rightFlywheel.getSetpoint()) < 10)
                && (Math.abs(m_rightChamber.getSetpoint() - m_setpoint) > 0.1))
                || m_rightChamber.isDisabled()) {
                m_rightChamber.updateSetpoint(m_setpoint);
            } else if ((Math.abs(m_rightChamber.getSetpoint() - m_setpoint) > 0.1)) {
                m_rightChamber.updateSetpoint(0);
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
        return Math.abs(m_rightChamber.getVelocity() - m_setpoint) < m_tolerance;
    }
}
