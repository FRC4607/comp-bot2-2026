// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.IntakeArmCalibrations;
import frc.robot.Constants.IntakeArmConstants;

/** Intake Arm subsystem. */
public class IntakeArm extends SubsystemBase {
    /** Creates a new IntakeArm. */

    private final TalonFX m_motor1;
    private final CANcoder m_encoder;

    private final TalonFXConfiguration m_talonFXConfig;
    private final CANcoderConfiguration m_encoderConfig;

    private final DynamicMotionMagicTorqueCurrentFOC m_request;

    /** Creates and configures the Intake Arm subsystem. */
    public IntakeArm() {

        m_motor1 = new TalonFX(IntakeArmConstants.kMotor1CANID, "kachow");
        m_encoder = new CANcoder(IntakeArmConstants.kEncoderCANID, "kachow");

        m_talonFXConfig = new TalonFXConfiguration();
        m_encoderConfig = new CANcoderConfiguration();

        m_request = new DynamicMotionMagicTorqueCurrentFOC(
                0.333,
                IntakeArmCalibrations.kMaxVelocity,
                IntakeArmCalibrations.kMaxAcceleration)
                .withJerk(IntakeArmCalibrations.kMaxJerk);

        // Feedback configs
        m_talonFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        m_talonFXConfig.Feedback.FeedbackRemoteSensorID = m_encoder.getDeviceID();
        m_talonFXConfig.Feedback.SensorToMechanismRatio = IntakeArmConstants.kSensorToMechanismRatio;
        m_talonFXConfig.Feedback.RotorToSensorRatio = IntakeArmConstants.kRotorToSensorRatio;

        m_talonFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        m_talonFXConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        m_talonFXConfig.Slot0.GravityArmPositionOffset = IntakeArmCalibrations.kGravityOffset;

        // Gains
        m_talonFXConfig.Slot0.kG = IntakeArmCalibrations.kG;
        m_talonFXConfig.Slot0.kS = IntakeArmCalibrations.kS;
        m_talonFXConfig.Slot0.kP = IntakeArmCalibrations.kP;
        m_talonFXConfig.Slot0.kI = IntakeArmCalibrations.kI;
        m_talonFXConfig.Slot0.kD = IntakeArmCalibrations.kD;

        // Current limit
        m_talonFXConfig.CurrentLimits.StatorCurrentLimit = IntakeArmCalibrations.kMaxAmperage;

        m_motor1.getConfigurator().apply(m_talonFXConfig);

        // Encoder offsets
        m_encoderConfig.MagnetSensor.MagnetOffset = IntakeArmCalibrations.kEncoderOffset;
        m_encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint
            = IntakeArmCalibrations.kEncoderDiscontinuityPoint;
        m_encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        m_encoder.getConfigurator().apply(m_encoderConfig);
    }

    public void runOpenLoop(double dutyCycle) {
        m_motor1.set(dutyCycle);
    }

    /**
     * Sets the setpoint of the mechanism in degrees.
     *
     * @param newSetpoint degrees (0, 80)
     */
    public void updateSetpoint(double newSetpoint) {
        m_motor1.setControl(m_request.withPosition(newSetpoint / 360));
    }

    public double getSetpoint() {
        return m_motor1.getClosedLoopReference().getValueAsDouble() * 360;
    }

    /**
     * Gets the position of the Intake Arm in degrees.
     *
     * @return the position of the mechanism in degrees
     */
    public double getPosition() {
        return m_encoder.getAbsolutePosition().getValueAsDouble() * 360;
    }

    @Override
    public void periodic() {

        // This method will be called once per scheduler run
    }
}
