// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import org.wpilib.command2.Command;
import frc.robot.subsystems.IntakeWheels;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/** SetIntakeWheelsOpenLoop command. */
public class SetIntakeWheelsOpenLoop extends Command {

    private DoubleSupplier m_dutyCycle;
    private IntakeWheels m_intakeWheels;

    /** 
     * A command to set the intake wheels power in open loop.
     *
     * @param dutyCycle The power to run the wheels at (-1, 1)
     * @param intakeWheels The intakeWheels to use
     */
    public SetIntakeWheelsOpenLoop(DoubleSupplier dutyCycle, IntakeWheels intakeWheels) {
        m_dutyCycle = dutyCycle;
        m_intakeWheels = intakeWheels;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_intakeWheels);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_intakeWheels.setOpenLoop(m_dutyCycle.getAsDouble());
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
