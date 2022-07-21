// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //note: all numeric values ARE NOT FINAL (except for seconds and basic geometry)
    //figure them out before testing or it will get very very ugly

    public static final class ModuleConstants {
        public static final double kWheelDiameterInches = 4; //! MEASURE
        public static final double kWheelDiameterMeters = Units.inchesToMeters(kWheelDiameterInches);
        public static final double kDriveMotorGearRatio = 1/6; //! MEASURE
        public static final double kTurningMotorGearRatio = 1/18; //! MEASURE
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters; 
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5; //! TUNE
    }
    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(21); //! MEASURE
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(25.5); //! MEASURE
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));
        //module parameters
        public static final int kFrontLeftDriveMotorPort = 8; //! PORT
        public static final int kBackLeftDriveMotorPort = 2;  //! PORT
        public static final int kFrontRightDriveMotorPort = 6; //! PORT
        public static final int kBackRightDriveMotorPort = 4; //! PORT

        public static final int kFrontLeftTurningMotorPort = 7; //! PORT
        public static final int kBackLeftTurningMotorPort = 1; //! PORT
        public static final int kFrontRightTurningMotorPort = 5; //! PORT
        public static final int kBackRightTurningMotorPort = 3; //! PORT

        public static final boolean kFrontLeftTurningEncoderReversed = true; //REVERSE
        public static final boolean kBackLeftTurningEncoderReversed = true;   //REVERSE
        public static final boolean kFrontRightTurningEncoderReversed = true; //REVERSE
        public static final boolean kBackRightTurningEncoderReversed = true; //REVERSE
 
        public static final boolean kFrontLeftDriveEncoderReversed = true; //REVERSE
        public static final boolean kBackLeftDriveEncoderReversed = true; //REVERSE
        public static final boolean kFrontRightDriveEncoderReversed = false; //REVERSE
        public static final boolean kBackRightDriveEncoderReversed = false; //REVERSE

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0; //PORT
        public static final int kBackLeftDriveAbsoluteEncoderPort = 2; //PORT
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1; //PORT
        public static final int kBackRightDriveAbsoluteEncoderPort = 3; //PORT

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false; //REVERSE
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false; //REVERSE
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false; //REVERSE
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false; //REVERSE


            //how many radians absolute encoder is offset; Once assemble, compare to straight wheels, determine values
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.254; //MEASURE
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -1.252;  //MEASURE
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -1.816; //MEASURE
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -4.811; //MEASURE

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5; //MAX
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI; //MAX ?

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3; //MAX
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3; //MAX
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5; //TUNE
        public static final double kPYController = 1.5; //TUNE
        public static final double kPThetaController = 3; //TUNE

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }


    public static final class OIConstants { 
        public static final int kDriverControllerPort = 0;  //PORT
 
        public static final int kDriverYAxis = 1; //AXIS
        public static final int kDriverXAxis = 0; //AXIS
        public static final int kDriverRotAxis = 4; //AXIS
        public static final int kDriverFieldOrientedButtonIdx = 1; //BUTTON
 
        public static final int kResetHeadingButton = 2; //BUTTON
 
        public static final double kDeadband = 0.05; 
    } 
}
