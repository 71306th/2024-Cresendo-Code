// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class JoystickConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final int leftStick_X = 0;
    public static final int leftStick_Y = 1;
    public static final int rightStick_X = 4;
    public static final int rightStick_Y = 5;
    public static final int trigger_L = 2;
    public static final int trigger_R = 3;
    public static final int btn_A = 1;
    public static final int btn_B = 2;
    public static final int btn_X = 3;
    public static final int btn_Y = 4;
    public static final int btn_LB = 5;
    public static final int btn_RB = 6;
    public static final int btn_LS = 9;
    public static final int btn_RS = 10;
  }

  public static final class Swerve {
    public static final double axisDeadBand = 0.05; // make sure ur robot won't vibrate cuz the joystick gives a input like 0.002 or sth
    public static final int pigeon = 5; // advanced gyro
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = 0.576; // meters, length between two side's wheels, need to adjust
    public static final double wheelBase = 0.576; // meters, length between same side's wheels, need to adjust
    public static final double wheelDiameter = Units.inchesToMeters(4.0); // need to adjust
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;  // open loop means no feedback(PID), closed loop vise versa, not used actually

    public static final double driveGearRatio = (6.7460317460317460317460317460317 / 1.0); // 6.75:1 (6.7460317460317460317460317460317), need to adjust
    public static final double angleGearRatio = (150.0 / 7.0 / 1.0); // 150/7 : 1, need to adjust

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)); // locating Swerve's positions, notice the sequences(first is 0, second is 1, etc.)

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0; // setting the nominal voltage(won't really follow anyway)

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 30; //20
    public static final int driveContinuousCurrentLimit = 40; //80, limiting the amps so Neo won't brake

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKD = 0.0; // maybe need to adjust

    /* Drive Motor PID Values */
    public static final double driveKP = 0.12;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0025;
    public static final double driveKFF = 0.0; // maybe need to adjust

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27; // feedforward, maybe need to adjust

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio; // like constants in physics

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.1; // meters per second
    public static final double maxAngularVelocity = 13.5; // meters per second

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake; // whether u want to let neo stop slowly individually(coast) or fiercely wholely(brake)

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = false; // yeah invert the motor

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false; // invert cancoder(in CTREconfig)

    /* Field Oriented */
    public static boolean fieldOriented = false;
    
    /* Slow Mode */
    public static boolean slow = false;
    public static double slowRegulator = 0.5;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 12;
      public static final int canCoderID = 1;
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-134.0);
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-131.572);
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-0.79);
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(135.5); // adjusted(2023/12/26)

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 21;
      public static final int angleMotorID = 22;
      public static final int canCoderID = 2;
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-6.2);
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-6.5);
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(137.5); // adjusted(2023/12/26)

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 31;
      public static final int angleMotorID = 32;
      public static final int canCoderID = 3;
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(15.21);
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(15.21);
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(1.5);
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(14.7); // adjusted(2023/12/26)

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }
    
    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 41;
      public static final int angleMotorID = 42;
      public static final int canCoderID = 4;
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-41.2);
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-39.882);
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-4);
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-99.2); // adjusted(2023/12/26)
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }
  }

  public static final class SuperStructure {
    /* Device IDs */
    public static final int tilterID = 5;
    public static final int floorMotorID = 6;
    public static final int uppermasterMotorID = 7;
    public static final int upperslaveMotorID = 8;
    public static final int limitSwitch = 1;

    /* Reserved Values */
    public static final double PodiumToSpeaker = 0; // need to adjust
    public static final double HumanIntake = 0; // need to adjust
    public static final double FloorIntake = 0; // need to adjust
    public static final double FloorPushNote = 0; // need to adjust
    public static final double IntakeIdle = 0;
    
    public static final double TilterPodiumToSpeaker = 0; // need to adjust
    public static final double TilterFloorIntake = 0;
    public static final double TilterHumanIntake = 0; // need to adjust
    public static final double TilterIdle = 0; // need to adjust

    public static final double falconEncooderCounts = 2048;

    public static boolean isAuto = false;
    public static boolean isFloor = false;

    /* Mathematical-Calculation-Requiring Values */
    public static final double IntakeMToTilter = 0; // intake's junction point with the tilter's arm to tilter, need to adjust
    public static final double IntakeMToIntakeU = 0; // intake's junction point to intake's upper motor, need to adjust
    public static final double IntakeUToTilterAngle = 0; // (intake's upper motor to tilter)'s angle, need to adjust
    public static final Translation3d TilterToBotCenter = new Translation3d(0, 0, 0); // need to adjust
    public static final double restrictedLowShootingAngle = 0; // need to adjust
    public static final double restrictedMaxMotorVelocity = 0; // need to adjust
  }

  public static final class Vision {
    public static final Pose3d CameraToBotCenter = new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    public static final double SpeakerIDToSpeakerZ = 0.795;
    public static final double SpeakerIDToSpeakerY = 0.565;
  }
} // all using meters