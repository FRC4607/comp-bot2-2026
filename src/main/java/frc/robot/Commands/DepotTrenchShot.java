// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations.DepotTrenchShotCalibrations;
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

/** The DepotTrenchShot sequential command group. */
public class DepotTrenchShot extends SequentialCommandGroup {
    /**
     * A command sequence to score from the depot-side trench.
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
    public DepotTrenchShot(LeftFlywheel leftFlywheel, LeftHood leftHood, LeftTurret leftTurret, Indexer indexer, LeftChamber leftChamber, RightFlywheel rightFlywheel, RightHood rightHood, RightTurret rightTurret, RightChamber rightChamber) {
        super(
            new ParallelCommandGroup(
                new LeftSetFlywheelVelocity(
                    () -> DepotTrenchShotCalibrations.kLeftFlywheelVelocity,
                    DepotTrenchShotCalibrations.kLeftFlywheelVelocityTolerance,
                    leftFlywheel).withTimeout(0.25),
                new LeftMoveHoodToPosition(
                    DepotTrenchShotCalibrations.kLeftHoodAngle,
                    DepotTrenchShotCalibrations.kLeftHoodAngleTolerance,
                    leftHood).withTimeout(0.25),
                new LeftMoveTurretToPosition(
                    () -> DepotTrenchShotCalibrations.kLeftTurretAngle,
                    DepotTrenchShotCalibrations.kLeftTurretAngleTolerance,
                    leftTurret).withTimeout(0.25),
                new RightSetFlywheelVelocity(
                    () -> DepotTrenchShotCalibrations.kRightFlywheelVelocity,
                    DepotTrenchShotCalibrations.kRightFlywheelVelocityTolerance,
                    rightFlywheel).withTimeout(0.25),
                new RightMoveHoodToPosition(
                    DepotTrenchShotCalibrations.kRightHoodAngle,
                    DepotTrenchShotCalibrations.kRightHoodAngleTolerance,
                    rightHood).withTimeout(0.25),
                new RightMoveTurretToPosition(
                    () -> DepotTrenchShotCalibrations.kRightTurretAngle,
                    DepotTrenchShotCalibrations.kRightTurretAngleTolerance,
                    rightTurret).withTimeout(0.25)),
            new ParallelCommandGroup(
                new SetIndexerVelocity(
                    DepotTrenchShotCalibrations.kIndexerVelocity,
                    DepotTrenchShotCalibrations.kIndexerVelocityTolerance,
                    indexer),
                new LeftSetChamberVelocity(
                    DepotTrenchShotCalibrations.kLeftChamberVelocity,
                    DepotTrenchShotCalibrations.kLeftChamberVelocityTolerance,
                    false, leftChamber, leftTurret, leftHood, leftFlywheel),
                new RightSetChamberVelocity(
                    DepotTrenchShotCalibrations.kRightChamberVelocity,
                    DepotTrenchShotCalibrations.kRightChamberVelocityTolerance,
                    false, rightChamber, rightTurret, rightHood, rightFlywheel))
        );
    }
}
