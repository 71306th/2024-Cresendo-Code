// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ChenryLib.MathUtility;
import frc.ChenryLib.PID;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  private final WPI_Pigeon2 gyro;

  private SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] mSwerveMods;
  private Vision m_vision;

  private Field2d field;

  private final PID adjustMidPID;
  private final PID adjustSidePID;

  public Swerve() {
    gyro = new WPI_Pigeon2(Constants.Swerve.pigeon, "7130");
    gyro.configFactoryDefault();
    zeroGyro();

    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), pos);

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

    m_vision = new Vision();

    field = new Field2d();

    adjustMidPID = new PID(0, 0, 0, 0, 0);
    adjustSidePID = new PID(0, 0, 0, 0, 0);
  }

  public static SwerveModulePosition[] pos = {
    new SwerveModulePosition(0, new Rotation2d(0)),
    new SwerveModulePosition(0, new Rotation2d(0)),
    new SwerveModulePosition(0, new Rotation2d(0)),
    new SwerveModulePosition(0, new Rotation2d(0))
  };

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
      fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw())
                    : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }


  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = mSwerveMods[0].getPosition();
    positions[1] = mSwerveMods[1].getPosition();
    positions[2] = mSwerveMods[2].getPosition();
    positions[3] = mSwerveMods[3].getPosition();
    return positions;
  }

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  public Rotation2d getYaw(){
    double averageAngle = gyro.getYaw();
    return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - averageAngle) : Rotation2d.fromDegrees(averageAngle);
  }

  public double getRoll(){
    return gyro.getRoll();
  }

  public void selectTab(String tab) {
    Shuffleboard.selectTab(tab);
  }

  public double calculateAutoFacing() {
    if(m_vision.getID() == 4 || m_vision.getID() == 7){
      double RotationVal = MathUtility.clamp(
      adjustMidPID.calculate(m_vision.getHorizontalDerivation()), 
      Constants.Swerve.slow ? -Math.pow(Constants.Swerve.slowRegulator, 2) : -Constants.Swerve.slowRegulator, 
      Constants.Swerve.slow ? Math.pow(Constants.Swerve.slowRegulator, 2) : Constants.Swerve.slowRegulator);
      return RotationVal;
    } else if(m_vision.getID() == 3 || m_vision.getID() == 8) {
      double RotationVal = MathUtility.clamp(
      adjustSidePID.calculate(m_vision.getHorizontalDerivation()), 
      Constants.Swerve.slow ? -Math.pow(Constants.Swerve.slowRegulator, 2) : -Constants.Swerve.slowRegulator, 
      Constants.Swerve.slow ? Math.pow(Constants.Swerve.slowRegulator, 2) : Constants.Swerve.slowRegulator);
      return RotationVal;
    } else return 0;
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getPositions());
    field.setRobotPose(getPose());

    SmartDashboard.putNumber("gyro ", getYaw().getDegrees());
    SmartDashboard.putBoolean("isOriented ", Constants.Swerve.fieldOriented);
    SmartDashboard.putBoolean("isSlow ", Constants.Swerve.slow);

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getAngle().getDegrees());

      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }

  // SmartDashboard.putNumber("ROLL", getFrontRoll());

  
  // System.out.println("Gyro: " + getYaw().getDegrees());
  // System.out.println(swerveOdometry.getPoseMeters().getY() + ", " + swerveOdometry.getPoseMeters().getX() + ", " + swerveOdometry.getPoseMeters().getRotation().getDegrees());
}
