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
import frc.robot.Calibrations.IntakeArmCalibrations;
import frc.robot.Calibrations.IntakeWheelCalibrations;
import frc.robot.Constants.IntakeWheelConstants;

/** Intake Wheels subsystem. */
public class IntakeWheels extends SubsystemBase {

    private final TalonFX m_motor1;

    private final TalonFXConfiguration m_talonFXConfig;

    private final MotionMagicVelocityTorqueCurrentFOC m_request;

    /** Creates and configures the Intake Wheels subsystem. */
    public IntakeWheels() {

        m_motor1 = new TalonFX(IntakeWheelConstants.kMotor1CANID, "kachow");

        m_talonFXConfig = new TalonFXConfiguration();

        m_request = new MotionMagicVelocityTorqueCurrentFOC(0);

        m_talonFXConfig.MotionMagic.MotionMagicAcceleration = IntakeWheelCalibrations.kMaxAcceleration;

        m_talonFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        m_talonFXConfig.Feedback.SensorToMechanismRatio = 1;

        m_talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        m_talonFXConfig.Slot0.kS = IntakeWheelCalibrations.kS;
        m_talonFXConfig.Slot0.kV = IntakeWheelCalibrations.kV;
        m_talonFXConfig.Slot0.kP = IntakeWheelCalibrations.kP;
        m_talonFXConfig.Slot0.kI = IntakeWheelCalibrations.kI;
        m_talonFXConfig.Slot0.kD = IntakeWheelCalibrations.kD;

        // Current limit
        m_talonFXConfig.CurrentLimits.StatorCurrentLimit = IntakeWheelCalibrations.kMaxAmperage;
        m_talonFXConfig.CurrentLimits.SupplyCurrentLimit = IntakeWheelCalibrations.kMaxSupplyAmperage;

        m_motor1.getConfigurator().apply(m_talonFXConfig);
    }

    /**
     * Sets the velocity setpoint of the intake wheels, in motor rotations per second.
     *
     * @param newSetpoint The new velocity (-90, 90)
     */
    public void updateSetpoint(double newSetpoint) {
        m_motor1.setControl(m_request.withVelocity(newSetpoint));
    }

    /**
     * Gets the current velocity setpoint of the intake wheels.
     *
     * @return The current velocity setpoint
     */
    public double getSetpoint() {
        return m_request.Velocity;
    }

    /**
     * Sets the open loop power of the intake wheels.
     *
     * @param dutyCycle Power to run at (-1, 1)
     */
    public void setOpenLoop(double dutyCycle) {
        m_motor1.set(dutyCycle);
    }

    /**
     * Gets the current velocity of the intake wheels, in motor rotations per second.
     *
     * @return The current velocity of the intake wheels
     */
    public double getVelocity() {
        return m_motor1.getVelocity().getValueAsDouble();
    }

    /**
     * Gets the current torque of the intake wheels motor.
     *
     * @return The current torque in Nm
     */
    public double getStatorCurrent() {
        return m_motor1.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
