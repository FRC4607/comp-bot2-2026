// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RightHood;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

/** ZeroHoodSequence command group for the right hood. */
public class RightZeroHoodSequence extends SequentialCommandGroup {

    /**
     * A command sequence for zeroing the right hood.
     *
     * @param rightHood The rightHood to use
     */
    public RightZeroHoodSequence(RightHood rightHood) {
        super(
            new RightSetHoodOpenLoop(() -> 0.2, rightHood).withTimeout(0.1),
            new RightZeroHood(rightHood));
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
    }
}
