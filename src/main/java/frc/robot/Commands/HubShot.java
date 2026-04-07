// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations.HubShotCalibrations;
import frc.robot.subsystems.LeftChamber;
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

/** HubShot sequential command group. */
public class HubShot extends SequentialCommandGroup {

    /**
     * A Command sequence that will shoot into the Hub when the robot is
     * touching the hub with the intake to the right.
     *
     * @param leftFlywheel The leftFlywheel to use
     * @param leftHood     The leftHood to use
     * @param leftTurret   The leftTurret to use
     * @param indexer  The indexer to use
     * @param leftChamber  The leftChamber to use
     * @param rightFlywheel The rightFlywheel to use
     * @param rightHood     The rightHood to use
     * @param rightTurret   The rightTurret to use
     * @param rightChamber  The rightChamber to use
     */
    public HubShot(LeftFlywheel leftFlywheel, LeftHood leftHood, LeftTurret leftTurret, Indexer indexer, LeftChamber leftChamber, RightFlywheel rightFlywheel, RightHood rightHood, RightTurret rightTurret, RightChamber rightChamber) {
        super(
            new ParallelCommandGroup(
                new LeftSetFlywheelVelocity(
                    () -> HubShotCalibrations.kLeftFlywheelVelocity,
                    HubShotCalibrations.kLeftFlywheelVelocityTolerance,
                    leftFlywheel),
                new LeftMoveHoodToPosition(
                    HubShotCalibrations.kLeftHoodAngle,
                    HubShotCalibrations.kLeftHoodAngleTolerance,
                    leftHood),
                new LeftMoveTurretToPosition(
                    () -> HubShotCalibrations.kLeftTurretAngle,
                    HubShotCalibrations.kLeftTurretAngleTolerance,
                    leftTurret),
                new RightSetFlywheelVelocity(
                    () -> HubShotCalibrations.kRightFlywheelVelocity,
                    HubShotCalibrations.kRightFlywheelVelocityTolerance,
                    rightFlywheel),
                new RightMoveHoodToPosition(
                    HubShotCalibrations.kRightHoodAngle,
                    HubShotCalibrations.kRightHoodAngleTolerance,
                    rightHood),
                new RightMoveTurretToPosition(
                    () -> HubShotCalibrations.kRightTurretAngle,
                    HubShotCalibrations.kRightTurretAngleTolerance,
                    rightTurret)),
            new ParallelCommandGroup(
                new SetIndexerVelocity(
                    HubShotCalibrations.kIndexerVelocity,
                    HubShotCalibrations.kIndexerVelocityTolerance,
                    indexer),
                new LeftSetChamberVelocity(
                    HubShotCalibrations.kLeftChamberVelocity,
                    HubShotCalibrations.kLeftChamberVelocityTolerance,
                    false, leftChamber, leftTurret, leftHood, leftFlywheel),
                new RightSetChamberVelocity(
                    HubShotCalibrations.kRightChamberVelocity,
                    HubShotCalibrations.kRightChamberVelocityTolerance,
                    false, rightChamber, rightTurret, rightHood, rightFlywheel))
        );
    }
}
