// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
public class GeneralShot extends ParallelCommandGroup {
    /** Creates a new GeneralShot. */
    public GeneralShot(CommandSwerveDrivetrain drivetrain, Indexer indexer, LeftChamber leftChamber, LeftTurret leftTurret, LeftHood leftHood, LeftFlywheel leftFlywheel, RightChamber rightChamber, RightTurret rightTurret, RightHood rightHood, RightFlywheel rightFlywheel) {

        super(
            new PointAtHub(drivetrain, leftTurret, leftHood, leftFlywheel, rightTurret, rightHood, rightFlywheel),
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                new ParallelCommandGroup(
                    new LeftSetChamberVelocity(90, 3, true, leftChamber, leftTurret, leftHood, leftFlywheel),
                    new RightSetChamberVelocity(90, 3, true, rightChamber, rightTurret, rightHood, rightFlywheel),
                    new SetIndexerVelocity(90, 3, indexer)
                )
            )
        );
    }
}
