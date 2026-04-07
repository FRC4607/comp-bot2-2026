// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import javax.naming.PartialResultException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations.PassWithGyroCalibrations;
import frc.robot.subsystems.LeftChamber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LeftFlywheel;
import frc.robot.subsystems.LeftHood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LeftTurret;
import frc.robot.subsystems.RightChamber;
import frc.robot.subsystems.RightFlywheel;
import frc.robot.subsystems.RightHood;
import frc.robot.subsystems.RightTurret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

/** PassWithGyro command group. */
public class PassWithGyro extends ParallelCommandGroup {
    
    /** Creates a new PassWithGyro. */
    public PassWithGyro(CommandSwerveDrivetrain drivetrain, Indexer indexer, LeftChamber leftChamber, LeftTurret leftTurret, LeftHood leftHood,
            LeftFlywheel leftFlywheel, RightChamber rightChamber, RightTurret rightTurret, RightHood rightHood, RightFlywheel rightFlywheel) {
        super(
                new ConditionalCommand(
                    new LeftMoveTurretToPosition(
                        () -> -drivetrain.getState().Pose.getRotation().getDegrees(),
                        0,
                        leftTurret)
                        .alongWith(new RightMoveTurretToPosition(
                            () -> -drivetrain.getState().Pose.getRotation().getDegrees(), 
                            0, 
                            rightTurret)), 
                    new LeftMoveTurretToPosition(
                        () -> -drivetrain.getState().Pose.getRotation().getDegrees() + 180, 
                        0, 
                        leftTurret)
                        .alongWith(
                            new RightMoveTurretToPosition(
                                () -> -drivetrain.getState().Pose.getRotation().getDegrees() + 180, 
                                0, 
                                rightTurret)), 
                    () -> DriverStation.getAlliance().get() == Alliance.Red),
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new LeftMoveHoodToPosition(
                                                PassWithGyroCalibrations.kLeftHoodAngle,
                                                PassWithGyroCalibrations.kLeftHoodTolerance,
                                                leftHood)
                                                .withTimeout(
                                                        PassWithGyroCalibrations.kLeftHoodTimeout),
                                        new LeftSetFlywheelVelocity(
                                                () -> PassWithGyroCalibrations.kLeftFlywheelVelocity,
                                                PassWithGyroCalibrations.kLeftFlywheelVelocityTolerance,
                                                leftFlywheel)
                                                .withTimeout(
                                                        PassWithGyroCalibrations.kLeftFlywheelTimeout)),
                                new ParallelCommandGroup(
                                        new LeftSetChamberVelocity(
                                                PassWithGyroCalibrations.kLeftChamberVelocity,
                                                PassWithGyroCalibrations.kLeftChamberVelocityTolerance,
                                                true, leftChamber, leftTurret, leftHood, leftFlywheel))),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new RightMoveHoodToPosition(
                                                PassWithGyroCalibrations.kLeftHoodAngle,
                                                PassWithGyroCalibrations.kLeftHoodTolerance,
                                                rightHood)
                                                .withTimeout(
                                                        PassWithGyroCalibrations.kLeftHoodTimeout),
                                        new RightSetFlywheelVelocity(
                                                () -> PassWithGyroCalibrations.kLeftFlywheelVelocity,
                                                PassWithGyroCalibrations.kLeftFlywheelVelocityTolerance,
                                                rightFlywheel)
                                                .withTimeout(
                                                        PassWithGyroCalibrations.kLeftFlywheelTimeout)),
                                new ParallelCommandGroup(
                                        new RightSetChamberVelocity(
                                                PassWithGyroCalibrations.kLeftChamberVelocity,
                                                PassWithGyroCalibrations.kLeftChamberVelocityTolerance,
                                                true, rightChamber, rightTurret, rightHood, rightFlywheel)))).withTimeout(0.5),
                    new SetIndexerVelocity(90, 10, indexer)));

        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
    }
}
