// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations.OutpostShotCalibrations;
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

/** OutpostShot sequential command group. */
public class OutpostShot extends SequentialCommandGroup {

    /**
     * A Command sequence that will score when the robot is touching the outpost
     * corner with the intake facing the tower.
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
    public OutpostShot(LeftFlywheel leftFlywheel, LeftHood leftHood, LeftTurret leftTurret, Indexer indexer, LeftChamber leftChamber, RightFlywheel rightFlywheel, RightHood rightHood, RightTurret rightTurret, RightChamber rightChamber) {
        super(
            // Spin up flywheels, move hoods, move turrets
            new ParallelCommandGroup(
                new LeftSetFlywheelVelocity(
                    () -> OutpostShotCalibrations.kLeftFlywheelVelocity,
                    OutpostShotCalibrations.kLeftFlywheelVelocityTolerance,
                    leftFlywheel).withTimeout(0.25),
                new LeftMoveHoodToPosition(
                    OutpostShotCalibrations.kLeftHoodAngle,
                    OutpostShotCalibrations.kLeftHoodAngleTolerance,
                    leftHood).withTimeout(0.25),
                new LeftMoveTurretToPosition(
                    () -> OutpostShotCalibrations.kLeftTurretAngle,
                    OutpostShotCalibrations.kLeftTurretAngleTolerance,
                    leftTurret).withTimeout(0.25),
                new RightSetFlywheelVelocity(
                    () -> OutpostShotCalibrations.kRightFlywheelVelocity,
                    OutpostShotCalibrations.kRightFlywheelVelocityTolerance,
                    rightFlywheel).withTimeout(0.25),
                new RightMoveHoodToPosition(
                    OutpostShotCalibrations.kRightHoodAngle,
                    OutpostShotCalibrations.kRightHoodAngleTolerance,
                    rightHood).withTimeout(0.25),
                new RightMoveTurretToPosition(
                    () -> OutpostShotCalibrations.kRightTurretAngle,
                    OutpostShotCalibrations.kRightTurretAngleTolerance,
                    rightTurret).withTimeout(0.25)),
            // Once turrets, flywheels and hoods are prepped, run indexer and chambers.
            new ParallelCommandGroup(
                new SetIndexerVelocity(
                    OutpostShotCalibrations.kIndexerVelocity,
                    OutpostShotCalibrations.kIndexerVelocityTolerance,
                    indexer),
                new LeftSetChamberVelocity(
                    OutpostShotCalibrations.kLeftChamberVelocity,
                    OutpostShotCalibrations.kLeftChamberVelocityTolerance,
                    false, leftChamber, leftTurret, leftHood, leftFlywheel),
                new RightSetChamberVelocity(
                    OutpostShotCalibrations.kRightChamberVelocity,
                    OutpostShotCalibrations.kRightChamberVelocityTolerance,
                    false, rightChamber, rightTurret, rightHood, rightFlywheel)));
    }
}
