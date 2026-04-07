// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.RightChamberCalibrations;
import frc.robot.Constants.RightChamberConstants;

/** Right Chamber subsystem. */
public class RightChamber extends SubsystemBase {

    private final TalonFX m_motor1;

    private final TalonFXConfiguration m_talonFXConfig;

    private final VelocityTorqueCurrentFOC m_request;

    private boolean m_disable = false;
    private boolean m_reverseWhenDisabled = true;

    /** Creates and configures settings for the right chamber. */
    public RightChamber() {

        // Chamber Motor
        m_motor1 = new TalonFX(RightChamberConstants.kMotor1CANID, "kachow");

        m_talonFXConfig = new TalonFXConfiguration();

        m_request = new VelocityTorqueCurrentFOC(0).withAcceleration(RightChamberCalibrations.kMaxAcceleration);

        // Feedback settings
        m_talonFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        m_talonFXConfig.Feedback.SensorToMechanismRatio = 1;

        m_talonFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Gains
        m_talonFXConfig.Slot0.kS = RightChamberCalibrations.kS;
        m_talonFXConfig.Slot0.kV = RightChamberCalibrations.kV;
        m_talonFXConfig.Slot0.kP = RightChamberCalibrations.kP;
        m_talonFXConfig.Slot0.kI = RightChamberCalibrations.kI;
        m_talonFXConfig.Slot0.kD = RightChamberCalibrations.kD;

        // Current limit
        m_talonFXConfig.CurrentLimits.StatorCurrentLimit = RightChamberCalibrations.kMaxAmperage;

        m_motor1.getConfigurator().apply(m_talonFXConfig);
    }

    public void updateSetpoint(double newSetpoint) {
        if (!m_disable) {
            m_motor1.setControl(m_request.withVelocity(newSetpoint));
        } else {
            if (m_reverseWhenDisabled) {
                m_motor1.setControl(m_request.withVelocity(-Math.abs(newSetpoint)));
            } else {
                m_motor1.set(0);
            }
        }
    }

    public double getSetpoint() {
        return m_motor1.getClosedLoopReference().getValueAsDouble();
    }

    public void runOpenLoop(double dutyCycle) {
        if (!m_disable) {
            m_motor1.set(dutyCycle);
        } else {
            if (m_reverseWhenDisabled) {
                m_motor1.set(-Math.abs(dutyCycle));
            } else {
                m_motor1.set(0);
            }
        }
    }

    public double getVelocity() {
        return m_motor1.getVelocity().getValueAsDouble();

    }

    public void disable(boolean disable) {
        m_disable = disable;
    }

    public void reverseWhenDisabled(boolean reverseWhenDisabled) {
        m_reverseWhenDisabled = reverseWhenDisabled;
    }

    public boolean isDisabled() {
        return m_disable;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
