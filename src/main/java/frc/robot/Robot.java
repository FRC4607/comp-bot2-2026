// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

        RobotController.setBrownoutVoltage(6.3);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        // Code to run every 0.02 seconds (20 milliseconds)
        if (!isDisabled()) {
            m_loopCounter++;

            var m_speeds = m_robotContainer.drivetrain.getState().Speeds.fromRobotRelativeSpeeds(
                    m_robotContainer.drivetrain.getState().Speeds,
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
                                0.7 + m_speeds.vxMetersPerSecond + (mrllMeasurement.avgTagDist / 2),
                                0.7 + m_speeds.vyMetersPerSecond + (mrllMeasurement.avgTagDist / 2),
                                9999999));
                m_hasTags = true;
            } else if (mrllMeasurement != null && mrllMeasurement.tagCount > 0 && (mrllMeasurement.avgTagDist < 2)) {
                m_robotContainer.drivetrain.addVisionMeasurement(
                        mrllMeasurement.pose,
                        mrllMeasurement.timestampSeconds,
                        VecBuilder.fill(
                                0.7 + m_speeds.vxMetersPerSecond
                                        + mrllMeasurement.avgTagDist,
                                0.7 + m_speeds.vyMetersPerSecond + mrllMeasurement.avgTagDist,
                                9999999));
                m_hasTags = true;
            }

            var brllMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-br");
            if (brllMeasurement != null && brllMeasurement.tagCount >= 2) {
                m_robotContainer.drivetrain.addVisionMeasurement(
                        brllMeasurement.pose,
                        brllMeasurement.timestampSeconds,
                        VecBuilder.fill(
                                0.7 + m_speeds.vxMetersPerSecond + (brllMeasurement.avgTagDist / 2),
                                0.7 + m_speeds.vyMetersPerSecond + (brllMeasurement.avgTagDist / 2),
                                9999999));
                m_hasTags = true;
            } else if (brllMeasurement != null && brllMeasurement.tagCount > 0 && (brllMeasurement.avgTagDist < 2)) {
                m_robotContainer.drivetrain.addVisionMeasurement(
                        brllMeasurement.pose, brllMeasurement.timestampSeconds,
                        VecBuilder.fill(
                                0.7 + m_speeds.vxMetersPerSecond + brllMeasurement.avgTagDist,
                                0.7 + m_speeds.vyMetersPerSecond + brllMeasurement.avgTagDist,
                                9999999));
                m_hasTags = true;
            }

            var blllMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-bl");
            if (blllMeasurement != null && blllMeasurement.tagCount >= 2) {
                m_robotContainer.drivetrain.addVisionMeasurement(
                        blllMeasurement.pose, blllMeasurement.timestampSeconds,
                        VecBuilder.fill(
                                0.7 + m_speeds.vxMetersPerSecond + (blllMeasurement.avgTagDist / 2),
                                0.7 + m_speeds.vyMetersPerSecond + (blllMeasurement.avgTagDist / 2),
                                9999999));
                m_hasTags = true;
            } else if (blllMeasurement != null && blllMeasurement.tagCount > 0 && (blllMeasurement.avgTagDist < 2)) {
                m_robotContainer.drivetrain.addVisionMeasurement(
                        blllMeasurement.pose,
                        blllMeasurement.timestampSeconds,
                        VecBuilder.fill(
                                0.7 + m_speeds.vxMetersPerSecond
                                        + blllMeasurement.avgTagDist,
                                0.7 + m_speeds.vyMetersPerSecond + blllMeasurement.avgTagDist,
                                9999999));
                m_hasTags = true;
            }

            var mlllMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-ml");
            if (mlllMeasurement != null && mlllMeasurement.tagCount >= 2) {
                m_robotContainer.drivetrain.addVisionMeasurement(
                        mlllMeasurement.pose, mlllMeasurement.timestampSeconds,
                        VecBuilder.fill(
                                0.7 + m_speeds.vxMetersPerSecond + (mlllMeasurement.avgTagDist / 2),
                                0.7 + m_speeds.vyMetersPerSecond + (mlllMeasurement.avgTagDist / 2),
                                9999999));
                m_hasTags = true;
            } else if (mlllMeasurement != null && mlllMeasurement.tagCount > 0 && (mlllMeasurement.avgTagDist < 2)) {
                m_robotContainer.drivetrain.addVisionMeasurement(
                        mlllMeasurement.pose,
                        mlllMeasurement.timestampSeconds,
                        VecBuilder.fill(
                                0.7 + m_speeds.vxMetersPerSecond
                                        + mlllMeasurement.avgTagDist,
                                0.7 + m_speeds.vyMetersPerSecond + mlllMeasurement.avgTagDist,
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
                SmartDashboard.putBoolean("Are Hoods Down?", m_robotContainer.m_leftHood.getPosition() < 0.5
                        && m_robotContainer.m_rightHood.getPosition() < 0.5);

                SmartDashboard.putBoolean("Has Tags?", m_hasTags);

                m_isHubActive = isHubActive();

                SmartDashboard.putBoolean("Hub State", m_isHubActive);
                SmartDashboard.putNumber("Time Until Switch", m_countDown);

                if (!DriverStation.getGameSpecificMessage().isBlank()) {
                    if (!m_isHubActive) {
                        if (m_countDown < 1) {
                            if (m_rumbleStage < 6) {
                                CommandScheduler.getInstance().schedule(
                                        new ParallelDeadlineGroup(
                                                new WaitCommand(0.5),
                                                new InstantCommand(
                                                        () -> m_robotContainer.joystick.setRumble(RumbleType.kBothRumble, 1)))
                                                .andThen(new InstantCommand(
                                                        () -> m_robotContainer.joystick.setRumble(RumbleType.kBothRumble, 0))));
                            }
                            m_rumbleStage = 6;
                        } else if (m_countDown < 2) {
                            if (m_rumbleStage < 5) {
                                CommandScheduler.getInstance().schedule(
                                        new ParallelDeadlineGroup(
                                                new WaitCommand(0.5),
                                                new InstantCommand(
                                                        () -> m_robotContainer.joystick.setRumble(RumbleType.kBothRumble, 1)))
                                                .andThen(new InstantCommand(
                                                        () -> m_robotContainer.joystick.setRumble(RumbleType.kBothRumble, 0))));
                            }
                            m_rumbleStage = 5;
                        } else if (m_countDown < 3) {
                            if (m_rumbleStage < 4) {
                                CommandScheduler.getInstance().schedule(
                                        new ParallelDeadlineGroup(
                                                new WaitCommand(0.5),
                                                new InstantCommand(
                                                        () -> m_robotContainer.joystick.setRumble(RumbleType.kBothRumble, 1)))
                                                .andThen(new InstantCommand(
                                                        () -> m_robotContainer.joystick.setRumble(RumbleType.kBothRumble, 0))));
                            }
                            m_rumbleStage = 4;
                        } else if (m_countDown < 4) {
                            if (m_rumbleStage < 3) {
                                CommandScheduler.getInstance().schedule(
                                        new ParallelDeadlineGroup(
                                                new WaitCommand(0.5),
                                                new InstantCommand(
                                                        () -> m_robotContainer.joystick.setRumble(RumbleType.kBothRumble, 1)))
                                                .andThen(new InstantCommand(
                                                        () -> m_robotContainer.joystick.setRumble(RumbleType.kBothRumble, 0))));
                            }
                            m_rumbleStage = 3;
                        } else if (m_countDown < 5) {
                            if (m_rumbleStage < 2) {
                                CommandScheduler.getInstance().schedule(
                                        new ParallelDeadlineGroup(
                                                new WaitCommand(0.5),
                                                new InstantCommand(
                                                        () -> m_robotContainer.joystick.setRumble(RumbleType.kBothRumble, 1)))
                                                .andThen(new InstantCommand(
                                                        () -> m_robotContainer.joystick.setRumble(RumbleType.kBothRumble, 0))));
                            }
                            m_rumbleStage = 2;
                        } else if (m_countDown < 10) {
                            if (m_rumbleStage < 1) {
                                CommandScheduler.getInstance().schedule(
                                        new ParallelDeadlineGroup(
                                                new WaitCommand(0.25),
                                                new InstantCommand(
                                                        () -> m_robotContainer.joystick.setRumble(RumbleType.kBothRumble, 1)))
                                                .andThen(new ParallelDeadlineGroup(
                                                        new WaitCommand(0.25),
                                                        new InstantCommand(() -> m_robotContainer.joystick
                                                                .setRumble(RumbleType.kBothRumble, 0))))
                                                .andThen(new ParallelDeadlineGroup(
                                                        new WaitCommand(0.25),
                                                        new InstantCommand(() -> m_robotContainer.joystick
                                                                .setRumble(RumbleType.kBothRumble, 1))))
                                                .andThen(new ParallelDeadlineGroup(
                                                        new WaitCommand(0.25),
                                                        new InstantCommand(() -> m_robotContainer.joystick
                                                                .setRumble(RumbleType.kBothRumble, 0))))
                                                .andThen(new ParallelDeadlineGroup(
                                                        new WaitCommand(0.25),
                                                        new InstantCommand(() -> m_robotContainer.joystick
                                                                .setRumble(RumbleType.kBothRumble, 1))))
                                                .andThen(new InstantCommand(() -> m_robotContainer.joystick.setRumble(RumbleType.kBothRumble, 0))));
                            }
                            m_rumbleStage = 1;
                        } else {
                            CommandScheduler.getInstance().cancel(m_robotContainer.RumblePulseFast);
                            m_robotContainer.joystick.setRumble(RumbleType.kBothRumble, 0);
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
                                                        () -> m_robotContainer.joystick.setRumble(RumbleType.kBothRumble, 1)))
                                                .andThen(new ParallelDeadlineGroup(
                                                        new WaitCommand(0.25),
                                                        new InstantCommand(() -> m_robotContainer.joystick
                                                                .setRumble(RumbleType.kBothRumble, 0))))
                                                .andThen(new ParallelDeadlineGroup(
                                                        new WaitCommand(0.25),
                                                        new InstantCommand(() -> m_robotContainer.joystick
                                                                .setRumble(RumbleType.kBothRumble, 1))))
                                                .andThen(new InstantCommand(() -> m_robotContainer.joystick.setRumble(RumbleType.kBothRumble, 0))));
                            }
                            m_rumbleStage = 1;
                        } else {
                            CommandScheduler.getInstance().cancel(m_robotContainer.RumblePulseFast);
                            m_robotContainer.joystick.setRumble(RumbleType.kBothRumble, 0);
                            m_rumbleStage = 0;
                        }
                    }
                }
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

            SmartDashboard.putString("Remaining Match Time",
                    LocalTime.of(0,
                            (int) Math.abs(DriverStation.getMatchTime() / 60),
                            (int) Math.abs(DriverStation.getMatchTime()) % 60).toString());
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

        if (DriverStation.isDSAttached()) {

            m_alliance = DriverStation.getAlliance().get();
            if (m_alliance != null) {
                if (m_robotContainer.getAutonomousCommand() == null) {
                    m_robotContainer.m_ledSubsystem.set(4, LEDSubsystem.kStrobeFastPurple);
                } else {
                    if (m_alliance == Alliance.Red) {
                        m_robotContainer.m_ledSubsystem.set(4, LEDSubsystem.kFadeRed);
                    } else {
                        m_robotContainer.m_ledSubsystem.set(4, LEDSubsystem.kFadeBlue);
                    }
                }
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
        LimelightHelpers.triggerRewindCapture("limelight-br", 170);
        LimelightHelpers.triggerRewindCapture("limelight-bl", 170);

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

        m_matchTime = DriverStation.getMatchTime();
        String m_gameData = DriverStation.getGameSpecificMessage();

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

        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (m_matchTime > 130) {
            m_countDown = m_matchTime - 130;
            return true;
        } else if (m_matchTime > 105) {

            if ((m_redActiveFirst && (alliance.get() == Alliance.Red)) || (!m_redActiveFirst && (alliance.get() == Alliance.Blue))) {
                m_countDown = m_matchTime - 105;
                return true;
            } else {
                m_countDown = m_matchTime - 105;
                return false;
            }

        } else if (m_matchTime > 80) {

            if ((m_redActiveFirst && (alliance.get() == Alliance.Red)) || (!m_redActiveFirst && (alliance.get() == Alliance.Blue))) {
                m_countDown = m_matchTime - 80;
                return false;
            } else {
                m_countDown = m_matchTime - 80;
                return true;
            }

        } else if (m_matchTime > 55) {

            if ((m_redActiveFirst && (alliance.get() == Alliance.Red)) || (!m_redActiveFirst && (alliance.get() == Alliance.Blue))) {
                m_countDown = m_matchTime - 55;
                return true;
            } else {
                m_countDown = m_matchTime - 55;
                return false;
            }

        } else if (m_matchTime > 30) {

            if ((m_redActiveFirst && (alliance.get() == Alliance.Red)) || (!m_redActiveFirst && (alliance.get() == Alliance.Blue))) {
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
