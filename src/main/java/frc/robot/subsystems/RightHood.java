// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.RightHoodCalibrations;
import frc.robot.Constants.RightHoodConstants;

/** Right Hood subsystem. */
public class RightHood extends SubsystemBase {

    private final TalonFXS m_motor;

    private final TalonFXSConfiguration m_talonFXSConfig;

    private final MotionMagicVoltage m_request;

    private boolean m_disable = false;

    /** Creates and configures the right hood subsystem. */
    public RightHood() {

        m_motor = new TalonFXS(RightHoodConstants.kMotorCANID, "kachow");

        m_talonFXSConfig = new TalonFXSConfiguration();

        m_request = new MotionMagicVoltage(0);

        // Select the motor type to use
        m_talonFXSConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        // TODO: Verify motor inversion direction for right hood
        m_talonFXSConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_talonFXSConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        // Gains
        m_talonFXSConfig.Slot0.kS = RightHoodCalibrations.kS;
        m_talonFXSConfig.Slot0.kP = RightHoodCalibrations.kP;
        m_talonFXSConfig.Slot0.kI = RightHoodCalibrations.kI;
        m_talonFXSConfig.Slot0.kD = RightHoodCalibrations.kD;

        // Motion Magic settings
        m_talonFXSConfig.MotionMagic.MotionMagicCruiseVelocity = RightHoodCalibrations.kMaxSpeed;
        m_talonFXSConfig.MotionMagic.MotionMagicAcceleration = RightHoodCalibrations.kMaxAcceleration;

        m_talonFXSConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Use the minion motor encoder as a feedback source
        m_talonFXSConfig.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.Commutation;

        // Current limit
        m_talonFXSConfig.CurrentLimits.StatorCurrentLimit = RightHoodCalibrations.kMaxAmperage;

        m_motor.getConfigurator().apply(m_talonFXSConfig);

    }

    // TODO: Update all position related methods to use degrees instead of motor rotations.
    /**
     * Gets the velocity of the hood, in motor rotations per second.
     *
     * @return the current velocity of the hood
     */
    public double getVelocity() {
        return m_motor.getVelocity().getValueAsDouble();
    }

    /**
     * Gets the position of the hood, in motor rotations.
     *
     * @return The current position of the hood
     */
    public double getPosition() {
        return m_motor.getPosition().getValueAsDouble();
    }

    /**
     * Sets the setpoint of the hood, in motor rotation.
     *
     * @param newSetpoint The position to drive towards (0, 2.2)
     */
    public void updateSetpoint(double newSetpoint) {
        if (!m_disable) {
            if (newSetpoint > 2.25) {
                newSetpoint = 2.25;
            }
            m_motor.setControl(m_request.withPosition(newSetpoint));
        } else {
            m_motor.set(0);
        }
    }

    /**
     * Gets the setpoint of the hood, in motor rotations.
     *
     * @return The setpoint of the motor (0, 2.25)
     */
    public double getSetpoint() {
        return m_motor.getClosedLoopReference().getValueAsDouble();
    }

    /**
     * Runs the hood in open loop.
     *
     * @param dutyCycle The power to run at (-1, 1)
     */
    public void runOpenLoop(double dutyCycle) {
        if (!m_disable) {
            m_motor.set(dutyCycle);
        } else {
            m_motor.set(0);
        }
    }

    /**
     * Resets the position of the hood.
     *
     * @param newPosition The updated position of the hood.
     */
    public void resetPosition(double newPosition) {
        m_motor.setPosition(newPosition);
    }

    public void disable(boolean disable) {
        m_disable = disable;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
