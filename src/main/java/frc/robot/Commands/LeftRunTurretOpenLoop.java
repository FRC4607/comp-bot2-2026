// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LeftTurret;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/** RunTurretOpenLoop command. */
public class LeftRunTurretOpenLoop extends Command {
    private DoubleSupplier m_dutyCycle;
    private LeftTurret m_LeftTurret;

    /**
     * A command that runs the turret in open loop control.
     *
     * @param dutyCycle The power to run at (-1, 1)
     * @param turret    The turret to use
     */
    public LeftRunTurretOpenLoop(DoubleSupplier dutyCycle, LeftTurret leftTurret) {
        m_dutyCycle = dutyCycle;
        m_LeftTurret = leftTurret;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_LeftTurret);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_LeftTurret.runOpenLoop(m_dutyCycle.getAsDouble());
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
