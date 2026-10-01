// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import org.wpilib.math.util.MathUtil;
import org.wpilib.command2.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/** WheelRadiusCalibration command. */
public class WheelRadiusCalibration extends Command {

  private double m_angle;
  private double m_driveStartPosition;
  private CommandSwerveDrivetrain m_drivetrain;
  private SwerveRequest.FieldCentric m_drive;

  /**
   * A command to calibrate the radius of the wheels.
   *
   * @param drivetrain The drivetrain to use
   * @param drive      The SwerveRequest to use
   */
  public WheelRadiusCalibration(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive) {
    m_drivetrain = drivetrain;
    m_drive = drive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_angle = m_drivetrain.getPigeon2().getYaw().getValueAsDouble();
    m_driveStartPosition = m_drivetrain.getModule(1).getDriveMotor().getPosition().getValueAsDouble();

    m_drivetrain.setControl(m_drive.withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(0.5));

    System.out.println("Started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Periodically print the target rate to rotate at.
    System.out.println("going at rate: " + m_drive.RotationalRate);

    // Periodically print the difference of the start angle and the current angle.
    System.out.println(m_angle - m_drivetrain.getPigeon2().getYaw().getValueAsDouble());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      // Stop the drivetrain when it ends.
      m_drivetrain.setControl(m_drive.withVelocityX(0)
          .withVelocityY(0)
          .withRotationalRate(0));

      // The final circumference of the wheels
      double circumference =
          // Calculates circumference of the path of the drive wheel
          ((Math.abs(m_drivetrain.getPigeon2().getYaw().getValueAsDouble() - m_angle) / 360)
              * (Math.PI * 2 * Math.sqrt(
                  Math.abs(MathUtil.copyDirectionPow(9.625 /*
                                                            * distance between FL to FR swerve modules
                                                            * divided by 2 in inches
                                                            */, 2))
                      + Math.abs(MathUtil.copyDirectionPow(9.625 /*
                                                                  * distance between FL to BL
                                                                  * swerve modules divided by 2 in
                                                                  * inches
                                                                  */, 2)))))
              // Divided by wheel rotations
              / (Math.abs(m_driveStartPosition
                  - m_drivetrain.getModule(1).getDriveMotor().getPosition().getValueAsDouble())
                  / TunerConstants.kDriveGearRatio);

      // Print the calculated circumference and radius to the Driver Station.
      System.out.println("Wheel Circumference: " + circumference);
      System.out.println("Wheel Radius" + (circumference / (Math.PI * 2)));
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_angle - m_drivetrain.getPigeon2().getYaw().getValueAsDouble()) > 1440;
  }
}
