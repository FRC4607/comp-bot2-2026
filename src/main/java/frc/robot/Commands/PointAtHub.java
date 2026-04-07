// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.Preferences;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LeftTurretConstants;
import frc.robot.Constants.RightTurretConstants;
import frc.robot.HoodInterpolatingTreeMap;
import frc.robot.FlywheelInterpolatingTreeMap;
import frc.robot.Robot;
import frc.robot.Calibrations.ShootingCalibrations;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LeftFlywheel;
import frc.robot.subsystems.LeftHood;
import frc.robot.subsystems.LeftTurret;
import frc.robot.subsystems.RightFlywheel;
import frc.robot.subsystems.RightHood;
import frc.robot.subsystems.RightTurret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PointAtHub extends Command {

    private CommandSwerveDrivetrain m_drivetrain;
    private LeftTurret m_leftTurret;
    private LeftHood m_leftHood;
    private LeftFlywheel m_leftFlywheel;
    private RightTurret m_rightTurret;
    private RightHood m_rightHood;
    private RightFlywheel m_rightFlywheel;

    private Translation2d m_targetHubPose;

    private Pose2d m_leftTurretPose;
    private Pose2d m_rightTurretPose;

    private Transform2d m_leftTurretTransform2d;
    private Transform2d m_rightTurretTransform2d;

    private HoodInterpolatingTreeMap m_hoodMap;
    private FlywheelInterpolatingTreeMap m_flywheelMap;

    private double m_drivetrainAngle;
    
    private double m_leftShotOffset;
    private double m_leftDistance;
    private double m_leftOffestHubX;
    private double m_leftOffsetHubY;

    private double m_rightShotOffset;
    private double m_rightDistance;
    private double m_rightOffestHubX;
    private double m_rightOffsetHubY;

    private ChassisSpeeds m_speeds;

    /** Creates a new PointAtHub. */
    public PointAtHub(CommandSwerveDrivetrain drivetrain, LeftTurret leftTurret, LeftHood leftHood, LeftFlywheel leftFlywheel, RightTurret rightTurret, RightHood rightHood, RightFlywheel rightFlywheel) {
        m_drivetrain = drivetrain;
        m_leftTurret = leftTurret;
        m_leftHood = leftHood;
        m_leftFlywheel = leftFlywheel;
        m_rightTurret = rightTurret;
        m_rightHood = rightHood;
        m_rightFlywheel = rightFlywheel;

        m_leftTurretTransform2d = new Transform2d(new Translation2d(-0.206375, 0.180975), new Rotation2d(0));
        m_rightTurretTransform2d = new Transform2d(new Translation2d(-0.206375, -0.180975), new Rotation2d(0));

        m_hoodMap = HoodInterpolatingTreeMap.createDefaultMap();
        m_flywheelMap = FlywheelInterpolatingTreeMap.createDefaultMap();
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_leftTurret, m_leftHood, m_leftFlywheel);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.get() == Alliance.Red) {
            m_targetHubPose = FieldConstants.kRedHub;
            System.out.println("Aiming at Red Hub");
        } else {
            m_targetHubPose = FieldConstants.kBlueHub;
            System.out.println("Aiming at Blue Hub");
        }
        
        m_drivetrainAngle = m_drivetrain.getState().Pose.getRotation().getDegrees();
    }
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_leftTurretPose = m_drivetrain.getState().Pose.plus(m_leftTurretTransform2d);
        m_rightTurretPose = m_drivetrain.getState().Pose.plus(m_rightTurretTransform2d);


        m_drivetrainAngle = m_drivetrain.getState().Pose.getRotation().getDegrees();

        m_speeds = m_drivetrain.getState().Speeds.fromRobotRelativeSpeeds(
            m_drivetrain.getState().Speeds, m_drivetrain.getState().Pose.getRotation());
            
        m_leftDistance = Math.hypot(
                m_leftTurretPose.getX() - m_targetHubPose.getX(), 
                m_leftTurretPose.getY() - m_targetHubPose.getY());
        
        m_leftOffestHubX = m_targetHubPose.getX() 
            - (m_speeds.vxMetersPerSecond * ShootingCalibrations.kVelocityOffsetMult 
            * ((ShootingCalibrations.kVelocityDistanceMult * m_leftDistance) + ShootingCalibrations.kVelocityDistanceConst));

        m_leftOffsetHubY = m_targetHubPose.getY() 
            - (m_speeds.vyMetersPerSecond * ShootingCalibrations.kVelocityOffsetMult 
            * ((ShootingCalibrations.kVelocityDistanceMult * m_leftDistance) + ShootingCalibrations.kVelocityDistanceConst));

        if (m_leftOffestHubX > m_drivetrain.getState().Pose.getX()) {
            m_leftShotOffset = 0;
        } else {
            m_leftShotOffset = 180;
        }


        m_leftTurret.updateSetpoint(-(
            (m_drivetrainAngle + m_leftShotOffset)

            /* ArcTangent to find field relative turret angle */
            - ((((Math.atan((m_leftTurretPose.getY() - m_leftOffsetHubY) 
            / (m_leftTurretPose.getX() - (m_leftOffestHubX))) 
            / Math.PI) * 180)))));

        m_leftFlywheel.updateSetpoint(Preferences.getDouble(ShootingCalibrations.kLeftFlywheelDistanceMultPrefKey, ShootingCalibrations.kLeftFlywheelDistanceMult) * m_flywheelMap.interpolate(
            Math.hypot(
                m_leftTurretPose.getX() - (m_leftOffestHubX), 
                m_leftTurretPose.getY() - (m_leftOffsetHubY))));

        m_leftHood.updateSetpoint(m_hoodMap.interpolate(
            Math.hypot(
                m_leftTurretPose.getX() - (m_leftOffestHubX), 
                m_leftTurretPose.getY() - (m_leftOffsetHubY))));

        // SmartDashboard.putNumber("Distance", m_leftDistance);
        // System.out.println(m_hoodMap.interpolate(
        //     Math.hypot(
        //         m_leftTurretPose.getX() - (m_leftOffestHubX), 
        //         m_leftTurretPose.getY() - (m_leftOffsetHubY))));

        // // Include the operater-entered value in the signal logger for checking later
        SignalLogger.writeDouble("Shooting/LeftFlywheelMult", SmartDashboard.getNumber(ShootingCalibrations.kLeftFlywheelDistanceMultPrefKey, ShootingCalibrations.kLeftFlywheelDistanceMult));

        m_rightDistance = Math.hypot(
            m_rightTurretPose.getX() - m_targetHubPose.getX(), 
            m_rightTurretPose.getY() - m_targetHubPose.getY());
        
        m_rightOffestHubX = m_targetHubPose.getX() 
            - (m_speeds.vxMetersPerSecond * ShootingCalibrations.kVelocityOffsetMult 
            * ((ShootingCalibrations.kVelocityDistanceMult * m_rightDistance) + ShootingCalibrations.kVelocityDistanceConst));

        m_rightOffsetHubY = m_targetHubPose.getY() 
            - (m_speeds.vyMetersPerSecond * ShootingCalibrations.kVelocityOffsetMult 
            * ((ShootingCalibrations.kVelocityDistanceMult * m_rightDistance) + ShootingCalibrations.kVelocityDistanceConst));

        if (m_rightOffestHubX > m_drivetrain.getState().Pose.getX()) {
            m_rightShotOffset = 0;
        } else {
            m_rightShotOffset = 180;
        }


        m_rightTurret.updateSetpoint(-(
            (m_drivetrainAngle + m_rightShotOffset)

            /* ArcTangent to find field relative turret angle */
            - ((((Math.atan((m_rightTurretPose.getY() - m_rightOffsetHubY) 
            / (m_rightTurretPose.getX() - (m_rightOffestHubX))) 
            / Math.PI) * 180)))));

        m_rightFlywheel.updateSetpoint(Preferences.getDouble(ShootingCalibrations.kRightFlywheelDistanceMultPrefKey, ShootingCalibrations.kRightFlywheelDistanceMult) * m_flywheelMap.interpolate(
            Math.hypot(
                m_rightTurretPose.getX() - (m_rightOffestHubX), 
                m_rightTurretPose.getY() - (m_rightOffsetHubY))));

        m_rightHood.updateSetpoint(m_hoodMap.interpolate(
            Math.hypot(
                m_rightTurretPose.getX() - (m_rightOffestHubX), 
                m_rightTurretPose.getY() - (m_rightOffsetHubY))));

        // Include the operater-entered value in the signal logger for checking later
        SignalLogger.writeDouble("Shooting/RightFlywheelMult", SmartDashboard.getNumber(ShootingCalibrations.kRightFlywheelDistanceMultPrefKey, ShootingCalibrations.kRightFlywheelDistanceMult));

        // SmartDashboard.putNumber("Right Turret Distance To Hub", m_rightDistance);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
