// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Calibrations.RightFlywheelCalibrations;
import frc.robot.Constants.RightFlywheelConstants;

/** Right Flywheel subsystem. */
public class RightFlywheel extends SubsystemBase {
    private final TalonFX m_motor1;
    private final TalonFX m_motor2;

    private TalonFXConfiguration m_talonFXConfig;

    private VelocityTorqueCurrentFOC m_request;

    private boolean m_disable = false;

    /** Creates and configures the right flywheel subsystem. */
    public RightFlywheel() {

        m_motor1 = new TalonFX(RightFlywheelConstants.kMotor1CANID, "kachow");
        m_motor2 = new TalonFX(RightFlywheelConstants.kMotor2CANID, "kachow");

        m_talonFXConfig = new TalonFXConfiguration();

        m_request = new VelocityTorqueCurrentFOC(0);

        // TODO: Verify motor inversion direction for right flywheel
        m_talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Gains
        m_talonFXConfig.Slot0.kS = RightFlywheelCalibrations.kS;
        m_talonFXConfig.Slot0.kV = RightFlywheelCalibrations.kV;
        m_talonFXConfig.Slot0.kP = RightFlywheelCalibrations.kP;
        m_talonFXConfig.Slot0.kI = RightFlywheelCalibrations.kI;
        m_talonFXConfig.Slot0.kD = RightFlywheelCalibrations.kD;

        m_talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Current limit
        m_talonFXConfig.CurrentLimits.StatorCurrentLimit = RightFlywheelCalibrations.kMaxAmperage;

        m_motor1.getConfigurator().apply(m_talonFXConfig);
        m_motor2.getConfigurator().apply(m_talonFXConfig);

        // TODO: Verify follower alignment for right flywheel
        m_motor2.setControl(new Follower(RightFlywheelConstants.kMotor1CANID, MotorAlignmentValue.Opposed));
    }

    public void updateSetpoint(double newSetpoint) {
        if (!m_disable) {
            m_motor1.setControl(m_request
                    .withVelocity(newSetpoint));
        } else {
            m_motor1.set(0);
        }
    }

    public double getSetpoint() {
        return m_motor1.getClosedLoopReference().getValueAsDouble();
    }

    public void runOpenLoop(double dutyCycle) {
        if (!m_disable) {
            m_motor1.set(dutyCycle);
        } else {
            m_motor1.set(0);
        }
    }

    public double getVelocity() {
        return m_motor1.getVelocity().getValueAsDouble();
    }

    public void disable(boolean disable) {
        m_disable = disable;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
