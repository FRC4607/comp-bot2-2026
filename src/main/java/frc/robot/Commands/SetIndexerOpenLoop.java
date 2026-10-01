// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import org.wpilib.command2.Command;
import frc.robot.subsystems.Indexer;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/** SetIndexerOpenLoop command. */
public class SetIndexerOpenLoop extends Command {
    private DoubleSupplier m_dutyCycle;
    private Indexer m_indexer;

    /** 
     * A command to set the power of the indexer in open loop.
     *
     * @param dutyCycle The power to run the motor at (-1, 1)
     * @param indexer The indexer to use
     */
    public SetIndexerOpenLoop(DoubleSupplier dutyCycle, Indexer indexer) {
        m_dutyCycle = dutyCycle;
        m_indexer = indexer;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_indexer.runOpenLoop(m_dutyCycle.getAsDouble());
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
