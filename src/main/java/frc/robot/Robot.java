// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Calibrations.ShootingCalibrations;
import frc.robot.Constants.FieldConstants;

import java.time.LocalTime;
import java.util.Optional;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private static Robot robotInstance;

    public final RobotContainer m_robotContainer;
    private int m_loopCounter;
    private double m_countDown;
    public Translation2d m_targetHubPose;
    public double m_shotOffset;

    public Robot() {
        robotInstance = this;

        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        // Code to run every 0.02 seconds (20 milliseconds)

        m_loopCounter++;

        var m_speeds = m_robotContainer.drivetrain.getState().Speeds.fromRobotRelativeSpeeds(m_robotContainer.drivetrain.getState().Speeds, m_robotContainer.drivetrain.getState().Pose.getRotation());

        // LimelightHelpers.SetRobotOrientation("limelight-br", m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees() + 180, 0, 0, 0, 0, 0);
        // LimelightHelpers.SetRobotOrientation("limelight-bl", m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees() + 180, 0, 0, 0, 0, 0);

        var brllMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-br");
        var blllMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-bl");
        if (brllMeasurement != null && brllMeasurement.tagCount >= 2) {
            m_robotContainer.drivetrain.addVisionMeasurement(brllMeasurement.pose, brllMeasurement.timestampSeconds, VecBuilder.fill(0.7 + m_speeds.vxMetersPerSecond, 0.7 + m_speeds.vyMetersPerSecond, 9999999));
        } else if (brllMeasurement != null && brllMeasurement.tagCount > 0 && (brllMeasurement.avgTagDist < 2)) {
            m_robotContainer.drivetrain.addVisionMeasurement(brllMeasurement.pose, brllMeasurement.timestampSeconds, VecBuilder.fill(0.7 + m_speeds.vxMetersPerSecond, 0.7 + m_speeds.vyMetersPerSecond, 9999999));
        }
        if (blllMeasurement != null && blllMeasurement.tagCount >= 2) {
            m_robotContainer.drivetrain.addVisionMeasurement(blllMeasurement.pose, blllMeasurement.timestampSeconds, VecBuilder.fill(0.7 + m_speeds.vxMetersPerSecond, 0.7 + m_speeds.vyMetersPerSecond, 9999999));
        } else if (blllMeasurement != null && blllMeasurement.tagCount > 0 && (blllMeasurement.avgTagDist < 2)) {
            m_robotContainer.drivetrain.addVisionMeasurement(blllMeasurement.pose, blllMeasurement.timestampSeconds, VecBuilder.fill(0.7 + m_speeds.vxMetersPerSecond, 0.7 + m_speeds.vyMetersPerSecond, 9999999));
        }
        

        // Code to run every 0.2 seconds (200 milliseconds)
        if ((m_loopCounter % 10) == 0) {
            SmartDashboard.putBoolean("Are Hoods Down?", m_robotContainer.m_leftHood.getPosition() < 0.5 && m_robotContainer.m_rightHood.getPosition() < 0.5);

            if ((brllMeasurement != null && brllMeasurement.tagCount > 0 
                    && (brllMeasurement.avgTagDist < 2 || brllMeasurement.tagCount >= 2)) 
                    || (blllMeasurement != null && blllMeasurement.tagCount > 0 
                    && (blllMeasurement.avgTagDist < 2 || blllMeasurement.tagCount >= 2))) {
                
                SmartDashboard.putBoolean("Has Tags?", true);
            } else {
                SmartDashboard.putBoolean("Has Tags?", false);
            }
        }

        // Code to run every 1 seconds (1000 milliseconds)
        if ((m_loopCounter % 50) == 0) {

            SmartDashboard.putNumber(ShootingCalibrations.kLeftFlywheelDistanceMultPrefKey, 
                Preferences.getDouble(ShootingCalibrations.kLeftFlywheelDistanceMultPrefKey, 
                    ShootingCalibrations.kLeftFlywheelDistanceMult));
                    
            SmartDashboard.putNumber(ShootingCalibrations.kRightFlywheelDistanceMultPrefKey, 
                Preferences.getDouble(ShootingCalibrations.kRightFlywheelDistanceMultPrefKey, 
                    ShootingCalibrations.kRightFlywheelDistanceMult));
            
            SmartDashboard.putBoolean("Hub State", isHubActive());
            SmartDashboard.putNumber("Time Until Switch", m_countDown);

            SmartDashboard.putString("Remaining Match Time", 
                LocalTime.of(0, 
                (int) Math.abs(DriverStation.getMatchTime() / 60), 
                (int) Math.abs(DriverStation.getMatchTime()) % 60).toString());

            SmartDashboard.putNumber(ShootingCalibrations.kFlywheelDistanceMultPrefKey, 
                Preferences.getDouble(ShootingCalibrations.kFlywheelDistanceMultPrefKey, 
                    ShootingCalibrations.kFlywheelDistanceMult));
        }
    }

    @Override
    public void disabledInit() {
        m_robotContainer.m_leftTurret.resetsetPosition();
        m_robotContainer.m_rightTurret.resetsetPosition();

        Preferences.initDouble(
            ShootingCalibrations.kLeftFlywheelDistanceMultPrefKey, ShootingCalibrations.kLeftFlywheelDistanceMult);
    }

    @Override
    public void disabledPeriodic() {
        
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {

        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.get() == Alliance.Red) {
            m_targetHubPose = FieldConstants.kRedHub;
        } else {
            m_targetHubPose = FieldConstants.kBlueHub;
        }

        m_robotContainer.m_intakeArm.updateSetpoint(m_robotContainer.m_intakeArm.getPosition());
        m_robotContainer.m_intakeWheels.updateSetpoint(0);
        m_robotContainer.m_indexer.runOpenLoop(0);
        m_robotContainer.m_leftChamber.runOpenLoop(0);
        m_robotContainer.m_leftTurret.runOpenLoop(0);
        m_robotContainer.m_leftHood.updateSetpoint(0);
        m_robotContainer.m_leftFlywheel.runOpenLoop(0);
        m_robotContainer.m_rightChamber.runOpenLoop(0);
        m_robotContainer.m_rightTurret.runOpenLoop(0);
        m_robotContainer.m_rightHood.updateSetpoint(0);
        m_robotContainer.m_rightFlywheel.runOpenLoop(0);

        // FMS Data Logging for debugging and post-match analysis
        SignalLogger.writeString("FMS/EventName", DriverStation.getEventName());
        SignalLogger.writeInteger("FMS/MatchNumber", DriverStation.getMatchNumber());
        SignalLogger.writeString("FMS/MatchType", DriverStation.getMatchType().toString());
        SignalLogger.writeInteger("FMS/ReplayNumber", DriverStation.getReplayNumber());
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
        SignalLogger.stop();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }

    public static Robot getRobotInstance() {
        return robotInstance;
    }

    public boolean isHubActive() {
        if (DriverStation.getAlliance().isEmpty()) {
            return false;
        }
        
        if (isAutonomousEnabled()) {
            return true;
        }

        if (!isTeleopEnabled()) {
            return false;
        }

        double m_matchTime = DriverStation.getMatchTime();
        String m_gameData = DriverStation.getGameSpecificMessage();

        if (m_gameData.isEmpty()) {
            return true;
        }

        boolean m_redActiveFirst;
        if (m_gameData.charAt(0) == 'B') {
            m_redActiveFirst = true;
        } else if (m_gameData.charAt(0) == 'A') {
            m_redActiveFirst = false;
        } else {
            return true;
        }

        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (m_matchTime > 130) {
            m_countDown = m_matchTime - 130;
            return true;
        } else if (m_matchTime > 105) {

            if (m_redActiveFirst && (alliance.get() == Alliance.Red)) {
                m_countDown = m_matchTime - 105;
                return true;
            } else {
                m_countDown = m_matchTime - 105;
                return false;
            }
            
        } else if (m_matchTime > 80) {

            if (m_redActiveFirst && (alliance.get() == Alliance.Red)) {
                m_countDown = m_matchTime - 80;
                return false;
            } else {
                m_countDown = m_matchTime - 80;
                return true;
            }

        } else if (m_matchTime > 55) {

            if (m_redActiveFirst && (alliance.get() == Alliance.Red)) {
                m_countDown = m_matchTime - 55;
                return true;
            } else {
                m_countDown = m_matchTime - 55;
                return false;
            }

        } else if (m_matchTime > 30) {

            if (m_redActiveFirst && (alliance.get() == Alliance.Red)) {
                m_countDown = m_matchTime - 30;
                return false;
            } else {
                m_countDown = m_matchTime - 30;
                return true;
            }

        } else {
            m_countDown = m_matchTime;
            return true;
        }
    }
    
}
