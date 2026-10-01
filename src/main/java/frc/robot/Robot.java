// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.telemetry.Telemetry;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.preferences.Preferences;
import org.wpilib.system.RobotController;
import org.wpilib.driverstation.Alliance;
import org.wpilib.framework.TimedRobot;
import org.wpilib.util.Color;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.ParallelDeadlineGroup;
import org.wpilib.command2.RepeatCommand;
import org.wpilib.command2.WaitCommand;
import frc.robot.Calibrations.ShootingCalibrations;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.LEDSubsystem;

import java.io.ObjectInputFilter.Config;
import java.time.Instant;
import java.time.LocalTime;
import java.util.Optional;
import java.util.Random;
import java.util.random.RandomGenerator;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private static Robot robotInstance;

  public final RobotContainer m_robotContainer;
  private int m_loopCounter;
  private double m_countDown;
  private boolean m_isHubActive;

  public Translation2d m_targetHubPose;
  public double m_shotOffset;

  public double m_matchTime;
  public int m_rumbleStage;

  private boolean m_hasTags;

  private boolean m_random;

  public Alliance m_alliance = null;

  public Robot() {
    robotInstance = this;

    m_robotContainer = new RobotContainer();
    m_random = new Random().nextBoolean();

    m_robotContainer.m_leftTurret.resetsetPosition();
    m_robotContainer.m_rightTurret.resetsetPosition();

    // MNSHL used a 6.3 V brownout. SystemCore also needs a recovery voltage at least
    // 0.5 V above it; 6.8 V is the lowest allowed. Alpha 6 didn't implement this and
    // threw from here (HAL error -1098, boot loop), so report a failure instead of crashing.
    try {
      RobotController.setBrownoutVoltages(6.3, 6.8);
    } catch (RuntimeException e) {
      DriverStationErrors.reportWarning("Could not set brownout voltages: " + e.getMessage(), false);
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.updateDashboard();

    // Code to run every 0.02 seconds (20 milliseconds)
    if (!isDisabled()) {
      m_loopCounter++;

      var m_speeds = m_robotContainer.drivetrain.getState().Velocity.toFieldRelative(
          m_robotContainer.drivetrain.getState().Pose.getRotation());

      // LimelightHelpers.SetRobotOrientation("limelight-br",
      // m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees() + 180,
      // 0, 0, 0, 0, 0);
      // LimelightHelpers.SetRobotOrientation("limelight-bl",
      // m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees() + 180,
      // 0, 0, 0, 0, 0);

      m_hasTags = false;

      var mrllMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-mr");
      if (mrllMeasurement != null && mrllMeasurement.tagCount >= 2) {
        m_robotContainer.drivetrain.addVisionMeasurement(
            mrllMeasurement.pose, mrllMeasurement.timestampSeconds,
            VecBuilder.fill(
                0.7 + m_speeds.vx + (mrllMeasurement.avgTagDist / 2),
                0.7 + m_speeds.vy + (mrllMeasurement.avgTagDist / 2),
                9999999));
        m_hasTags = true;
      } else if (mrllMeasurement != null && mrllMeasurement.tagCount > 0 && (mrllMeasurement.avgTagDist < 2)) {
        m_robotContainer.drivetrain.addVisionMeasurement(
            mrllMeasurement.pose,
            mrllMeasurement.timestampSeconds,
            VecBuilder.fill(
                0.7 + m_speeds.vx
                    + mrllMeasurement.avgTagDist,
                0.7 + m_speeds.vy + mrllMeasurement.avgTagDist,
                9999999));
        m_hasTags = true;
      }

      var brllMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-br");
      if (brllMeasurement != null && brllMeasurement.tagCount >= 2) {
        m_robotContainer.drivetrain.addVisionMeasurement(
            brllMeasurement.pose,
            brllMeasurement.timestampSeconds,
            VecBuilder.fill(
                0.7 + m_speeds.vx + (brllMeasurement.avgTagDist / 2),
                0.7 + m_speeds.vy + (brllMeasurement.avgTagDist / 2),
                9999999));
        m_hasTags = true;
      } else if (brllMeasurement != null && brllMeasurement.tagCount > 0 && (brllMeasurement.avgTagDist < 2)) {
        m_robotContainer.drivetrain.addVisionMeasurement(
            brllMeasurement.pose, brllMeasurement.timestampSeconds,
            VecBuilder.fill(
                0.7 + m_speeds.vx + brllMeasurement.avgTagDist,
                0.7 + m_speeds.vy + brllMeasurement.avgTagDist,
                9999999));
        m_hasTags = true;
      }

      var blllMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-bl");
      if (blllMeasurement != null && blllMeasurement.tagCount >= 2) {
        m_robotContainer.drivetrain.addVisionMeasurement(
            blllMeasurement.pose, blllMeasurement.timestampSeconds,
            VecBuilder.fill(
                0.7 + m_speeds.vx + (blllMeasurement.avgTagDist / 2),
                0.7 + m_speeds.vy + (blllMeasurement.avgTagDist / 2),
                9999999));
        m_hasTags = true;
      } else if (blllMeasurement != null && blllMeasurement.tagCount > 0 && (blllMeasurement.avgTagDist < 2)) {
        m_robotContainer.drivetrain.addVisionMeasurement(
            blllMeasurement.pose,
            blllMeasurement.timestampSeconds,
            VecBuilder.fill(
                0.7 + m_speeds.vx
                    + blllMeasurement.avgTagDist,
                0.7 + m_speeds.vy + blllMeasurement.avgTagDist,
                9999999));
        m_hasTags = true;
      }

      var mlllMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-ml");
      if (mlllMeasurement != null && mlllMeasurement.tagCount >= 2) {
        m_robotContainer.drivetrain.addVisionMeasurement(
            mlllMeasurement.pose, mlllMeasurement.timestampSeconds,
            VecBuilder.fill(
                0.7 + m_speeds.vx + (mlllMeasurement.avgTagDist / 2),
                0.7 + m_speeds.vy + (mlllMeasurement.avgTagDist / 2),
                9999999));
        m_hasTags = true;
      } else if (mlllMeasurement != null && mlllMeasurement.tagCount > 0 && (mlllMeasurement.avgTagDist < 2)) {
        m_robotContainer.drivetrain.addVisionMeasurement(
            mlllMeasurement.pose,
            mlllMeasurement.timestampSeconds,
            VecBuilder.fill(
                0.7 + m_speeds.vx
                    + mlllMeasurement.avgTagDist,
                0.7 + m_speeds.vy + mlllMeasurement.avgTagDist,
                9999999));
        m_hasTags = true;
      }

      if (m_hasTags) {
        if (!m_robotContainer.m_leftTurret.isDisabled()) {
          m_robotContainer.m_ledSubsystem.set(1, LEDSubsystem.kStrobeGreen);
        }
        if (!m_robotContainer.m_rightTurret.isDisabled()) {
          m_robotContainer.m_ledSubsystem.set(3, LEDSubsystem.kStrobeGreen);
        }
        m_robotContainer.m_ledSubsystem.set(2, LEDSubsystem.kStrobeGreen);
      } else {
        if (!m_robotContainer.m_leftTurret.isDisabled()) {
          m_robotContainer.m_ledSubsystem.set(1, LEDSubsystem.kSolidWhite);
        }
        if (!m_robotContainer.m_rightTurret.isDisabled()) {
          m_robotContainer.m_ledSubsystem.set(3, LEDSubsystem.kSolidWhite);
        }
        m_robotContainer.m_ledSubsystem.set(2, LEDSubsystem.kSolidWhite);
      }

      // Code to run every 0.2 seconds (200 milliseconds)
      if ((m_loopCounter % 10) == 0) {
        Telemetry.log("Are Hoods Down?", m_robotContainer.m_leftHood.getPosition() < 0.5
            && m_robotContainer.m_rightHood.getPosition() < 0.5);

        Telemetry.log("Has Tags?", m_hasTags);

        m_isHubActive = isHubActive();

        Telemetry.log("Hub State", m_isHubActive);
        Telemetry.log("Time Until Switch", m_countDown);

        if (!MatchState.getGameData().orElse("").isBlank()) {
          if (!m_isHubActive) {
            if (m_countDown < 1) {
              if (m_rumbleStage < 6) {
                CommandScheduler.getInstance().schedule(
                    new ParallelDeadlineGroup(
                        new WaitCommand(0.5),
                        new InstantCommand(
                            () -> m_robotContainer.setRumbleBoth(1)))
                        .andThen(new InstantCommand(
                            () -> m_robotContainer.setRumbleBoth(0))));
              }
              m_rumbleStage = 6;
            } else if (m_countDown < 2) {
              if (m_rumbleStage < 5) {
                CommandScheduler.getInstance().schedule(
                    new ParallelDeadlineGroup(
                        new WaitCommand(0.5),
                        new InstantCommand(
                            () -> m_robotContainer.setRumbleBoth(1)))
                        .andThen(new InstantCommand(
                            () -> m_robotContainer.setRumbleBoth(0))));
              }
              m_rumbleStage = 5;
            } else if (m_countDown < 3) {
              if (m_rumbleStage < 4) {
                CommandScheduler.getInstance().schedule(
                    new ParallelDeadlineGroup(
                        new WaitCommand(0.5),
                        new InstantCommand(
                            () -> m_robotContainer.setRumbleBoth(1)))
                        .andThen(new InstantCommand(
                            () -> m_robotContainer.setRumbleBoth(0))));
              }
              m_rumbleStage = 4;
            } else if (m_countDown < 4) {
              if (m_rumbleStage < 3) {
                CommandScheduler.getInstance().schedule(
                    new ParallelDeadlineGroup(
                        new WaitCommand(0.5),
                        new InstantCommand(
                            () -> m_robotContainer.setRumbleBoth(1)))
                        .andThen(new InstantCommand(
                            () -> m_robotContainer.setRumbleBoth(0))));
              }
              m_rumbleStage = 3;
            } else if (m_countDown < 5) {
              if (m_rumbleStage < 2) {
                CommandScheduler.getInstance().schedule(
                    new ParallelDeadlineGroup(
                        new WaitCommand(0.5),
                        new InstantCommand(
                            () -> m_robotContainer.setRumbleBoth(1)))
                        .andThen(new InstantCommand(
                            () -> m_robotContainer.setRumbleBoth(0))));
              }
              m_rumbleStage = 2;
            } else if (m_countDown < 10) {
              if (m_rumbleStage < 1) {
                CommandScheduler.getInstance().schedule(
                    new ParallelDeadlineGroup(
                        new WaitCommand(0.25),
                        new InstantCommand(
                            () -> m_robotContainer.setRumbleBoth(1)))
                        .andThen(new ParallelDeadlineGroup(
                            new WaitCommand(0.25),
                            new InstantCommand(() -> m_robotContainer
                                .setRumbleBoth(0))))
                        .andThen(new ParallelDeadlineGroup(
                            new WaitCommand(0.25),
                            new InstantCommand(() -> m_robotContainer
                                .setRumbleBoth(1))))
                        .andThen(new ParallelDeadlineGroup(
                            new WaitCommand(0.25),
                            new InstantCommand(() -> m_robotContainer
                                .setRumbleBoth(0))))
                        .andThen(new ParallelDeadlineGroup(
                            new WaitCommand(0.25),
                            new InstantCommand(() -> m_robotContainer
                                .setRumbleBoth(1))))
                        .andThen(
                            new InstantCommand(() -> m_robotContainer.setRumbleBoth(0))));
              }
              m_rumbleStage = 1;
            } else {
              CommandScheduler.getInstance().cancel(m_robotContainer.RumblePulseFast);
              m_robotContainer.setRumbleBoth(0);
              m_rumbleStage = 0;
            }
          } else {
            if (m_countDown < 5) {
              if (m_rumbleStage < 2) {
                CommandScheduler.getInstance().schedule(m_robotContainer.RumblePulseFast);
              }
              m_rumbleStage = 2;
            } else if (m_countDown < 10) {
              if (m_rumbleStage < 1) {
                CommandScheduler.getInstance().schedule(
                    new ParallelDeadlineGroup(
                        new WaitCommand(0.25),
                        new InstantCommand(
                            () -> m_robotContainer.setRumbleBoth(1)))
                        .andThen(new ParallelDeadlineGroup(
                            new WaitCommand(0.25),
                            new InstantCommand(() -> m_robotContainer
                                .setRumbleBoth(0))))
                        .andThen(new ParallelDeadlineGroup(
                            new WaitCommand(0.25),
                            new InstantCommand(() -> m_robotContainer
                                .setRumbleBoth(1))))
                        .andThen(
                            new InstantCommand(() -> m_robotContainer.setRumbleBoth(0))));
              }
              m_rumbleStage = 1;
            } else {
              CommandScheduler.getInstance().cancel(m_robotContainer.RumblePulseFast);
              m_robotContainer.setRumbleBoth(0);
              m_rumbleStage = 0;
            }
          }
        }
      }
    }

    // Code to run every 1 seconds (1000 milliseconds)
    if ((m_loopCounter % 50) == 0) {

      Telemetry.log(ShootingCalibrations.kLeftFlywheelDistanceMultPrefKey,
          Preferences.getDouble(ShootingCalibrations.kLeftFlywheelDistanceMultPrefKey,
              ShootingCalibrations.kLeftFlywheelDistanceMult));

      Telemetry.log(ShootingCalibrations.kRightFlywheelDistanceMultPrefKey,
          Preferences.getDouble(ShootingCalibrations.kRightFlywheelDistanceMultPrefKey,
              ShootingCalibrations.kRightFlywheelDistanceMult));

      Telemetry.log("Remaining Match Time",
          LocalTime.of(0,
              (int) Math.abs(MatchState.getMatchTime() / 60),
              (int) Math.abs(MatchState.getMatchTime()) % 60).toString());
    }
  }

  @Override
  public void disabledInit() {
    // Max 96
    // candle.setControl(new SolidColor(0, 3).withColor(kWhite));
    // candle.setControl(new SolidColor(4, 7).withColor(kWhite));
    // candle.setControl(new SolidColor(8, 28).withColor(kWhite));
    // candle.setControl(new SolidColor(29, 48).withColor(kWhite));
    // candle.setControl(new SolidColor(49, 68).withColor(kWhite));
    // candle.setControl(new SolidColor(69, 88).withColor(kWhite));
    // candle.setControl(new SolidColor(89, 96).withColor(kWhite));

    // m_robotContainer.m_leftTurret.resetsetPosition();
    // m_robotContainer.m_rightTurret.resetsetPosition();

    Preferences.initDouble(
        ShootingCalibrations.kLeftFlywheelDistanceMultPrefKey, ShootingCalibrations.kLeftFlywheelDistanceMult);
  }

  @Override
  public void disabledPeriodic() {

    if (RobotState.isDSAttached()) {

      m_alliance = MatchState.getAlliance().orElse(null);
      if (!m_robotContainer.isAutoSelected()) {
        // No auto selected ("None"): fast purple strobe, regardless of alliance
        m_robotContainer.m_ledSubsystem.set(4, LEDSubsystem.kStrobeFastPurple);
      } else if (m_alliance == Alliance.RED) {
        m_robotContainer.m_ledSubsystem.set(4, LEDSubsystem.kFadeRed);
      } else if (m_alliance != null) {
        m_robotContainer.m_ledSubsystem.set(4, LEDSubsystem.kFadeBlue);
      }

    } else {
      m_robotContainer.m_ledSubsystem.set(4, LEDSubsystem.kFadePurple);
    }
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
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

    Optional<Alliance> alliance = MatchState.getAlliance();
    if (alliance.get() == Alliance.RED) {
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
    SignalLogger.writeString("FMS/EventName", MatchState.getEventName());
    SignalLogger.writeInteger("FMS/MatchNumber", MatchState.getMatchNumber());
    SignalLogger.writeString("FMS/MatchType", MatchState.getMatchType().toString());
    SignalLogger.writeInteger("FMS/ReplayNumber", MatchState.getReplayNumber());
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
    LimelightHelpers.triggerRewindCapture("limelight-br", 170);
    LimelightHelpers.triggerRewindCapture("limelight-bl", 170);

    SignalLogger.stop();
  }

  @Override
  public void utilityInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void utilityPeriodic() {
  }

  @Override
  public void utilityExit() {
  }

  @Override
  public void simulationPeriodic() {
  }

  public static Robot getRobotInstance() {
    return robotInstance;
  }

  public boolean isHubActive() {
    if (MatchState.getAlliance().isEmpty()) {
      return false;
    }

    if (isAutonomousEnabled()) {
      return true;
    }

    if (!isTeleopEnabled()) {
      return false;
    }

    m_matchTime = MatchState.getMatchTime();
    String m_gameData = MatchState.getGameData().orElse("");

    boolean m_redActiveFirst;
    if (m_gameData.isEmpty()) {
      return true;
    } else {
      if (m_gameData.charAt(0) == 'B') {
        m_redActiveFirst = true;
      } else if (m_gameData.charAt(0) == 'A') {
        m_redActiveFirst = false;
      } else {
        m_redActiveFirst = m_random;
      }
    }

    Optional<Alliance> alliance = MatchState.getAlliance();

    if (m_matchTime > 130) {
      m_countDown = m_matchTime - 130;
      return true;
    } else if (m_matchTime > 105) {

      if ((m_redActiveFirst && (alliance.get() == Alliance.RED))
          || (!m_redActiveFirst && (alliance.get() == Alliance.BLUE))) {
        m_countDown = m_matchTime - 105;
        return true;
      } else {
        m_countDown = m_matchTime - 105;
        return false;
      }

    } else if (m_matchTime > 80) {

      if ((m_redActiveFirst && (alliance.get() == Alliance.RED))
          || (!m_redActiveFirst && (alliance.get() == Alliance.BLUE))) {
        m_countDown = m_matchTime - 80;
        return false;
      } else {
        m_countDown = m_matchTime - 80;
        return true;
      }

    } else if (m_matchTime > 55) {

      if ((m_redActiveFirst && (alliance.get() == Alliance.RED))
          || (!m_redActiveFirst && (alliance.get() == Alliance.BLUE))) {
        m_countDown = m_matchTime - 55;
        return true;
      } else {
        m_countDown = m_matchTime - 55;
        return false;
      }

    } else if (m_matchTime > 30) {

      if ((m_redActiveFirst && (alliance.get() == Alliance.RED))
          || (!m_redActiveFirst && (alliance.get() == Alliance.BLUE))) {
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
