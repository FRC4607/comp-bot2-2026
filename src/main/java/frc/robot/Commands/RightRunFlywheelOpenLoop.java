// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RightFlywheel;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/** RunFlywheelOpenLoop command for the right flywheel. */
public class RightRunFlywheelOpenLoop extends Command {

    private DoubleSupplier m_dutyCycle;
    private RightFlywheel m_rightFlywheel;

    /**
     * A command that runs the right flywheel in open loop control.
     *
     * @param dutyCycle The power to run at (-1, 1)
     * @param rightFlywheel  The rightFlywheel to use
     */
    public RightRunFlywheelOpenLoop(DoubleSupplier dutyCycle, RightFlywheel rightFlywheel) {
        m_dutyCycle = dutyCycle;
        m_rightFlywheel = rightFlywheel;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_rightFlywheel);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_rightFlywheel.runOpenLoop(m_dutyCycle.getAsDouble());
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
