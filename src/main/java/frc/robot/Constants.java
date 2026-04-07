package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation2d;

public class Constants {

    public class FieldConstants {
        /** Translation of the hub on the blue side. */
        public static final Translation2d kBlueHub = new Translation2d(Inches.of(182.143595), Inches.of(158.84375));

        /** Translation of the hub on the red side. */
        public static final Translation2d kRedHub = new Translation2d(Inches.of(469.078905), Inches.of(158.84375));
    }

    public class DrivetrainConstants {
        
    }

    public class IntakeArmConstants {

        /** CAN ID of the first motor. */
        public static final int kMotor1CANID = 15;

        /** CAN ID of the encoder. */
        public static final int kEncoderCANID = 15;

        /** How many sensor rotations equal one mechanism rotation. */
        public static final double kSensorToMechanismRatio = 2;

        /** How many motor rotations equal one sensor rotation. */
        public static final double kRotorToSensorRatio = 23;
    }

    public class IntakeWheelConstants {
        public static final int kMotor1CANID = 14;
    }

    public class IndexerConstants {

        /** CAN ID of the motor. */
        public static final int kMotor1CANID = 13;
    }

    /** Constants for the Left Chamber. */
    public class LeftChamberConstants {

        /** CAN ID of the first motor. */
        public static final int kMotor1CANID = 6;
    }

    /** Constants for the Left Turret. */
    public class LeftTurretConstants {

        /** CAN ID of the motor. */
        public static final int kMotorCANID = 7;

        /** CAN ID of the first encoder. */
        public static final int kEncoder1CANID = 31;

        /** CAN ID of the second encoder. */
        public static final int kEncoder2CANID = 32;

        /** Gear ratio of # of motor rotor rotations to one mechanism rotation. */
        public static final double kRotorToMechanism = 10.2;

        /** Gear ratio of # of sensor rotations to one mechanism rotation. */
        public static final double kEncoder1ToMechanism = 6.375;

        /** Gear ratio of # of motor rotor rotations to one sensor rotation. */
        public static final double kEncoder1ToRotor = 1.6;

        /** Gear ratio of # of sensor rotations to one mechanism rotation. */
        public static final double kEncoder2ToMechanism = 6.8;

        /** How far (Degrees) the turret is away from the right plane. */
        public static final double kLeftTurretPositionYaw = -50.29;

        /** How far (Meters) the turret is away from the center of the robot on the XY plane. */
        public static final double kLeftTurretHypotenuse = 0.2744978;
        
    }

    /** Constants for the minion on the left turret. */
    public class LeftHoodConstants {

        /** CAN ID of the motor. */
        public static final int kMotorCANID = 8;
    }

    /** Constants for the leftFlywheel on the left turret. */
    public class LeftFlywheelConstants {

        /** CAN ID of the first motor. */
        // TODO: Conflict with Turret kEncoder2CANID
        public static final int kMotor1CANID = 4;

        /** CAN ID of the second motor. */
        // TODO: Conflict with Turret kEncoder1CANID
        public static final int kMotor2CANID = 5;
    }

    /** Constants for the Right Chamber. */
    public class RightChamberConstants {

        /** CAN ID of the first motor. */
        // TODO: Set correct CAN ID
        public static final int kMotor1CANID = 17;
    }

    /** Constants for the Right Turret. */
    public class RightTurretConstants {

        /** CAN ID of the motor. */
        // TODO: Set correct CAN ID
        public static final int kMotorCANID = 16;

        /** CAN ID of the first encoder. */
        // TODO: Set correct CAN ID
        public static final int kEncoder1CANID = 41;

        /** CAN ID of the second encoder. */
        // TODO: Set correct CAN ID
        public static final int kEncoder2CANID = 42;

        /** Gear ratio of # of motor rotor rotations to one mechanism rotation. */
        public static final double kRotorToMechanism = 10.2;

        /** Gear ratio of # of sensor rotations to one mechanism rotation. */
        public static final double kEncoder1ToMechanism = 6.375;

        /** Gear ratio of # of motor rotor rotations to one sensor rotation. */
        public static final double kEncoder1ToRotor = 1.6;

        /** Gear ratio of # of sensor rotations to one mechanism rotation. */
        public static final double kEncoder2ToMechanism = 6.8;

        /** How far (Degrees) the turret is away from the right plane. */
        // TODO: Measure and set correct value
        public static final double kRightTurretPositionYaw = 50.29;

        /** How far (Meters) the turret is away from the center of the robot on the XY plane. */
        // TODO: Measure and set correct value
        public static final double kRightTurretHypotenuse = 0.2744978;

    }

    /** Constants for the hood on the right turret. */
    public class RightHoodConstants {

        /** CAN ID of the motor. */
        // TODO: Set correct CAN ID
        public static final int kMotorCANID = 9;
    }

    /** Constants for the flywheel on the right turret. */
    public class RightFlywheelConstants {

        /** CAN ID of the first motor. */
        // TODO: Set correct CAN ID
        public static final int kMotor1CANID = 19;

        /** CAN ID of the second motor. */
        // TODO: Set correct CAN ID
        public static final int kMotor2CANID = 18;
    }
}
