// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import org.wpilib.command2.Command;
import frc.robot.subsystems.IntakeArm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/** MoveIntakeToPosition command. */
public class MoveIntakeToPosition extends Command {

    private final double m_setpoint;
    private final double m_tolerance;
    private final IntakeArm m_intakeArm;

    /**
     * A command to set the setpoint of the intake Arm, in degrees.
     *
     * @param setpoint       The setpoint to drive towards (degrees)
     * @param tolerance      Tolerance for error (degrees)
     * @param intakeArm Intake Arm to use.
     */
    public MoveIntakeToPosition(double setpoint, double tolerance, IntakeArm intakeArm) {
        m_setpoint = setpoint;
        m_tolerance = tolerance;
        m_intakeArm = intakeArm;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_intakeArm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_intakeArm.updateSetpoint(m_setpoint);
        System.out.println("New Intake Arm Setpoint: " + m_setpoint);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("Intake arm interrupted");
        } else {
            System.out.println("Intake Arm finished moving to position");
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(m_setpoint - m_intakeArm.getPosition()) < m_tolerance;
    }
}
