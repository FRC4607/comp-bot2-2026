// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LeftChamber;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/** SetChamberOpenLoop command. */
public class LeftSetChamberOpenLoop extends Command {
    private DoubleSupplier m_dutyCycle;
    private LeftChamber m_leftChamber;

    /** Creates a new SetChamberOpenLoop. */
    public LeftSetChamberOpenLoop(DoubleSupplier dutyCycle, LeftChamber leftChamber) {
        m_leftChamber = leftChamber;
        m_dutyCycle = dutyCycle;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_leftChamber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_leftChamber.runOpenLoop(m_dutyCycle.getAsDouble());
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
