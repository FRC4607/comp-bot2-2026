// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations.OutpostTrenchShotCalibrations;
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

/** OutpostTrenchShot sequential command group. */
public class OutpostTrenchShot extends SequentialCommandGroup {
    /**
     * A command sequence to score from the outpost-side trench.
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
    public OutpostTrenchShot(LeftFlywheel leftFlywheel, LeftHood leftHood, LeftTurret leftTurret, Indexer indexer, LeftChamber leftChamber, RightFlywheel rightFlywheel, RightHood rightHood, RightTurret rightTurret, RightChamber rightChamber) {
        super(
            // Spin up flywheels, move hoods, move turrets
            new ParallelCommandGroup(
                new LeftSetFlywheelVelocity(
                    () -> OutpostTrenchShotCalibrations.kLeftFlywheelVelocity,
                    OutpostTrenchShotCalibrations.kLeftFlywheelVelocityTolerance,
                    leftFlywheel).withTimeout(0.25),
                new LeftMoveHoodToPosition(
                    OutpostTrenchShotCalibrations.kLeftHoodAngle,
                    OutpostTrenchShotCalibrations.kLeftHoodAngleTolerance,
                    leftHood).withTimeout(0.25),
                new LeftMoveTurretToPosition(
                    () -> OutpostTrenchShotCalibrations.kLeftTurretAngle,
                    OutpostTrenchShotCalibrations.kLeftTurretAngleTolerance,
                    leftTurret).withTimeout(0.25),
                new RightSetFlywheelVelocity(
                    () -> OutpostTrenchShotCalibrations.kRightFlywheelVelocity,
                    OutpostTrenchShotCalibrations.kRightFlywheelVelocityTolerance,
                    rightFlywheel).withTimeout(0.25),
                new RightMoveHoodToPosition(
                    OutpostTrenchShotCalibrations.kRightHoodAngle,
                    OutpostTrenchShotCalibrations.kRightHoodAngleTolerance,
                    rightHood).withTimeout(0.25),
                new RightMoveTurretToPosition(
                    () -> OutpostTrenchShotCalibrations.kRightTurretAngle,
                    OutpostTrenchShotCalibrations.kRightTurretAngleTolerance,
                    rightTurret).withTimeout(0.25)),
            // Once turrets, flywheels and hoods are prepped, run indexer and chambers.
            new ParallelCommandGroup(
                new SetIndexerVelocity(
                    OutpostTrenchShotCalibrations.kIndexerVelocity,
                    OutpostTrenchShotCalibrations.kIndexerVelocityTolerance,
                    indexer),
                new LeftSetChamberVelocity(
                    OutpostTrenchShotCalibrations.kLeftChamberVelocity,
                    OutpostTrenchShotCalibrations.kLeftChamberVelocityTolerance,
                    false, leftChamber, leftTurret, leftHood, leftFlywheel),
                new RightSetChamberVelocity(
                    OutpostTrenchShotCalibrations.kRightChamberVelocity,
                    OutpostTrenchShotCalibrations.kRightChamberVelocityTolerance,
                    false, rightChamber, rightTurret, rightHood, rightFlywheel)));
    }
}
