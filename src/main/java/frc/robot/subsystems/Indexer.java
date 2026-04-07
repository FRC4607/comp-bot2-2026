// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.IndexerCalibrations;
import frc.robot.Calibrations.IntakeArmCalibrations;
import frc.robot.Constants.IndexerConstants;

/** Indexer subsystem. */
public class Indexer extends SubsystemBase {

    private final TalonFX m_motor1;

    private final TalonFXConfiguration m_talonFXConfig;

    private final MotionMagicVelocityTorqueCurrentFOC m_request;

    /** Creates and configures the indexer subsystem. */
    public Indexer() {

        m_motor1 = new TalonFX(IndexerConstants.kMotor1CANID, "kachow");

        m_talonFXConfig = new TalonFXConfiguration();

        m_request = new MotionMagicVelocityTorqueCurrentFOC(0);

        m_talonFXConfig.MotionMagic.MotionMagicAcceleration = IndexerCalibrations.kMaxAcceleration;

        // Feedback configs
        m_talonFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        m_talonFXConfig.Feedback.SensorToMechanismRatio = 1;

        m_talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Gains
        m_talonFXConfig.Slot0.kS = IndexerCalibrations.kS;
        m_talonFXConfig.Slot0.kV = IndexerCalibrations.kV;
        m_talonFXConfig.Slot0.kP = IndexerCalibrations.kP;
        m_talonFXConfig.Slot0.kI = IndexerCalibrations.kI;
        m_talonFXConfig.Slot0.kD = IndexerCalibrations.kD;

        // Current limit
        m_talonFXConfig.CurrentLimits.StatorCurrentLimit = IndexerCalibrations.kMaxAmperage;

        m_motor1.getConfigurator().apply(m_talonFXConfig);
    }

    /**
     * Sets the velocity setpoint of the mechanism, in motor rotations per second.
     *
     * @param newSetpoint The new setpoint (-90, 90)
     */
    public void updateSetpoint(double newSetpoint) {
        m_motor1.setControl(m_request.withVelocity(newSetpoint));
    }

    /**
     * Runs the indexer in open loop control.
     *
     * @param dutyCycle The power to run at (-1, 1)
     */
    public void runOpenLoop(double dutyCycle) {
        m_motor1.set(dutyCycle);
    }

    /**
     * Gets the velocity of the indexer, in motor rotations per second.
     *
     * @return The current velocity of the indexer
     */
    public double getVelocity() {
        return m_motor1.getVelocity().getValueAsDouble();

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
