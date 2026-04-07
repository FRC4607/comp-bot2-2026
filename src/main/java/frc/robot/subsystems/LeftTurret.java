// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.IntakeArmCalibrations;
import frc.robot.Calibrations.LeftTurretCalibrations;
import frc.robot.Constants.LeftTurretConstants;

/** Turret subsystem. */
public class LeftTurret extends SubsystemBase {

    private final TalonFX m_motor;

    private final TalonFXConfiguration m_talonFXConfig;

    private final MotionMagicTorqueCurrentFOC m_request;

    private final CANcoder m_encoder1;
    private final CANcoder m_encoder2;

    private final CANcoderConfiguration m_encoderConfig1;
    private final CANcoderConfiguration m_encoderConfig2;

    private double m_position;

    private boolean m_disable = false;

    /** Creates and configures the turret subsystem. */
    public LeftTurret() {

        m_motor = new TalonFX(LeftTurretConstants.kMotorCANID, "kachow");

        m_talonFXConfig = new TalonFXConfiguration();

        m_request = new MotionMagicTorqueCurrentFOC(0);

        m_encoder1 = new CANcoder(LeftTurretConstants.kEncoder1CANID, "kachow");
        m_encoder2 = new CANcoder(LeftTurretConstants.kEncoder2CANID, "kachow");

        m_encoderConfig1 = new CANcoderConfiguration();
        m_encoderConfig2 = new CANcoderConfiguration();

        m_talonFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        m_talonFXConfig.Feedback.SensorToMechanismRatio = LeftTurretConstants.kRotorToMechanism;

        m_talonFXConfig.ClosedLoopGeneral.ContinuousWrap = false;

        m_talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_talonFXConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        m_talonFXConfig.Slot0.GravityArmPositionOffset = LeftTurretCalibrations.kGravityOffset;

        m_talonFXConfig.Slot0.kG = LeftTurretCalibrations.kG;
        m_talonFXConfig.Slot0.kS = LeftTurretCalibrations.kS;
        m_talonFXConfig.Slot0.kP = LeftTurretCalibrations.kP;
        m_talonFXConfig.Slot0.kI = LeftTurretCalibrations.kI;
        m_talonFXConfig.Slot0.kD = LeftTurretCalibrations.kD;

        m_talonFXConfig.MotionMagic.MotionMagicCruiseVelocity = LeftTurretCalibrations.kMaxSpeed;
        m_talonFXConfig.MotionMagic.MotionMagicAcceleration = LeftTurretCalibrations.kMaxAcceleration;
        m_talonFXConfig.MotionMagic.MotionMagicJerk = LeftTurretCalibrations.kMaxJerk;

        m_talonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        m_talonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = LeftTurretCalibrations.kForwardSoftLimit;

        m_talonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        m_talonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = LeftTurretCalibrations.kReverseSoftLimit;

        m_talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Current limit
        m_talonFXConfig.CurrentLimits.StatorCurrentLimit = LeftTurretCalibrations.kMaxAmperage;

        m_motor.getConfigurator().apply(m_talonFXConfig);

        m_encoderConfig1.MagnetSensor.MagnetOffset = LeftTurretCalibrations.kEncoder1Offset;
        m_encoderConfig1.MagnetSensor.AbsoluteSensorDiscontinuityPoint = LeftTurretCalibrations.kEncoder1Discontinuity;
        m_encoderConfig1.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        m_encoder1.getConfigurator().apply(m_encoderConfig1);

        m_encoderConfig2.MagnetSensor.MagnetOffset = LeftTurretCalibrations.kEncoder2Offset;
        m_encoderConfig2.MagnetSensor.AbsoluteSensorDiscontinuityPoint = LeftTurretCalibrations.kEncoder2Discontinuity;
        m_encoderConfig2.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        m_encoder2.getConfigurator().apply(m_encoderConfig2);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // SmartDashboard.putNumber("Turret Encoder Position", getPosition());
        // SmartDashboard.putNumber("Turret Motor Position", m_motor.getPosition().getValueAsDouble() * 360);
        // SmartDashboard.putNumber("Encoder 1", m_encoder1.getAbsolutePosition().getValueAsDouble());
        // SmartDashboard.putNumber("Encoder 2", m_encoder2.getAbsolutePosition().getValueAsDouble());
    }

    /**
     * Updates the position to drive towards.
     *
     * @param newSetpoint The new setpoint, in mechanism rotations.
     */
    public void updateSetpoint(double newSetpoint) {

        if (!m_disable) {
            // Converts incoming setpoint to rotations, modulus to get it close to zero, 
            // add one in case of negative, modulus by one again in case of a positive.
            m_motor.setControl(m_request.withPosition(((((newSetpoint + LeftTurretCalibrations.kClosedLoopOffset) / 360) % 1) + 1) % 1));
        } else {
            m_motor.set(0);
        }
    }

    /**
     * Runs the turret motor in open loop.
     *
     * @param dutyCycle The power to drive the motor at.
     */
    public void runOpenLoop(double dutyCycle) {
        if (!m_disable) {
            m_motor.set(dutyCycle);
        } else {
            m_motor.set(0);
        }
    }

    /**
     * Gets the current position of the turret in Mechanism rotations.
     * This method uses the Chinese Remainder Theorem to calculate the position of the turret from
     * two encoders with a one-tooth difference in their gears.
     *
     * @return the current position of the turret
     */
    public double getPosition() {
        m_position = (m_encoder1.getAbsolutePosition().getValueAsDouble())
                - (m_encoder2.getAbsolutePosition().getValueAsDouble());

        if (m_position >= 0) {
            return (m_position * 2.35) * 360;
        } else {
            return ((m_position + 1) * 2.35) * 360;
        }
    }

    /**
     * Gets the target position of the mechanism, in degrees.
     *
     * @return The setpoint in degrees.
     */
    public double getSetpoint() {
        return m_motor.getClosedLoopReference().getValueAsDouble() * 360;
    }

    /**
     * Resets the position of the turret to match what the encoders say.
     */
    public void resetsetPosition() {
        if ((getPosition() / 360) > 2) {
            m_motor.setPosition((getPosition() / 360) - 2.35);
        } else {
            m_motor.setPosition(getPosition() / 360);
        }
    }

    public void disable(boolean disable) {
        m_disable = disable;
    }

}
