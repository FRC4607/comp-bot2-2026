// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RightHood;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/** ZeroHood command for the right hood. */
public class RightZeroHood extends Command {
    private RightHood m_rightHood;
    private boolean m_isHoodAtVelocity;

    /**
     * A command to zero the right hood when it is away from the hard stop.
     *
     * @param rightHood The rightHood to use
     */
    public RightZeroHood(RightHood rightHood) {
        m_rightHood = rightHood;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_rightHood);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_isHoodAtVelocity = false;
        m_rightHood.runOpenLoop(-0.2);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if ((m_rightHood.getVelocity() < -0.3) & !m_isHoodAtVelocity) {
            m_isHoodAtVelocity = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            m_rightHood.resetPosition(0);
        }

        m_rightHood.runOpenLoop(0);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if ((m_rightHood.getVelocity() > -0.3) && m_isHoodAtVelocity) {
            return true;
        } else {
            return false;
        }
    }
}
