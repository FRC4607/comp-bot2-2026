// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RightHood;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/** SetHoodOpenLoop command for the right hood. */
public class RightSetHoodOpenLoop extends Command {
    private DoubleSupplier m_dutyCycle;
    private RightHood m_rightHood;

    /**
     * A command to set the power of the right hood in open loop.
     *
     * @param dutyCycle The power to run the motor at (-1, 1)
     * @param rightHood The rightHood to use
     */
    public RightSetHoodOpenLoop(DoubleSupplier dutyCycle, RightHood rightHood) {
        m_dutyCycle = dutyCycle;
        m_rightHood = rightHood;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_rightHood);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_rightHood.runOpenLoop(m_dutyCycle.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
