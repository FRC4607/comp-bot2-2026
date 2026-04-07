// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RightTurret;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/** RunTurretOpenLoop command for the right turret. */
public class RightRunTurretOpenLoop extends Command {
    private DoubleSupplier m_dutyCycle;
    private RightTurret m_rightTurret;

    /**
     * A command that runs the right turret in open loop control.
     *
     * @param dutyCycle The power to run at (-1, 1)
     * @param rightTurret    The right turret to use
     */
    public RightRunTurretOpenLoop(DoubleSupplier dutyCycle, RightTurret rightTurret) {
        m_dutyCycle = dutyCycle;
        m_rightTurret = rightTurret;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_rightTurret);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_rightTurret.runOpenLoop(m_dutyCycle.getAsDouble());
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
