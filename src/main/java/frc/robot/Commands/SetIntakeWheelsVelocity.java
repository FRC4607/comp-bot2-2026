// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Calibrations.IntakeWheelCalibrations;
import frc.robot.subsystems.IntakeWheels;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/** SetIntakeWheelsVelocity command. */
public class SetIntakeWheelsVelocity extends Command {

    private double m_setpoint;
    private double m_tolerance;
    private IntakeWheels m_intakeWheels;

    private double[] m_statorCurrentArray = new double[IntakeWheelCalibrations.kCurrentLimitSamplings];
    private int m_statorCurrentIndex;
    private double m_statorCurrentAvg;

    /** Creates a new SetIntakeWheelsVelocity. */
    public SetIntakeWheelsVelocity(double setpoint, double tolerance, IntakeWheels intakeWheels) {
        m_setpoint = setpoint;
        m_tolerance = tolerance;
        m_intakeWheels = intakeWheels;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_intakeWheels);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_intakeWheels.updateSetpoint(m_setpoint);

        Arrays.fill(m_statorCurrentArray, 0);
        m_statorCurrentIndex = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Store the current stator current of the intake motor in the array and update the index
        m_statorCurrentArray[m_statorCurrentIndex] = m_intakeWheels.getStatorCurrent();
        m_statorCurrentIndex = (m_statorCurrentIndex + 1) % m_statorCurrentArray.length; // Wrap around when reaching the end

        // gets the mean last 10 samples of StatorCurrent from the intake motor.
        m_statorCurrentAvg = 0;
        for (int i = 0; i < m_statorCurrentArray.length - 1; i++) {
            m_statorCurrentAvg += m_statorCurrentArray[i];
        }
        m_statorCurrentAvg /= m_statorCurrentArray.length;

        // Check to see if the average stator current is above the threshold
        if (m_statorCurrentAvg > IntakeWheelCalibrations.kMeanCurrentLimit) {
            // Check to see if the current setpoint is not near 10 rps
            if (Math.abs(m_intakeWheels.getSetpoint()) > 0.1) {
                // on true, set intake wheels to 10rps, copying the direction of the setpoint.
                m_intakeWheels.updateSetpoint(Math.copySign(0, m_setpoint));
            }
        } else {
            // Check to see if the current setpoint is not near the desired setpoint
            if (Math.abs(m_intakeWheels.getSetpoint() - m_setpoint) > 0.1) {
                m_intakeWheels.updateSetpoint(m_setpoint);
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intakeWheels.updateSetpoint(10);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
