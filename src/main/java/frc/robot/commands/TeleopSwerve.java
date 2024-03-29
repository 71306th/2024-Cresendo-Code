package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class TeleopSwerve extends CommandBase {

  private final Vision m_vision;
  private final Swerve s_Swerve;
  private final Controller m_controller;
  
  private XboxController driver;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  private boolean onePress1 = false;
  private boolean onePress2 = false;

  private double translationVal;
  private double strafeVal;
  private double rotationVal;

  public TeleopSwerve(Vision m_vision, Swerve s_Swerve, Controller m_controller) {
    this.m_vision = m_vision;
    addRequirements(m_vision);
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    this.m_controller = m_controller;
    addRequirements(m_controller);
    driver = m_controller.getDriverController();
  }

  @Override
  public void execute() {
    /* Get Values, Deadband*/
    if (Constants.Swerve.slow) {
      translationVal =
          translationLimiter.calculate(
              MathUtil.applyDeadband(driver.getLeftY() * Constants.Swerve.slowRegulator, Constants.Swerve.axisDeadBand));
      strafeVal =
          strafeLimiter.calculate(
              MathUtil.applyDeadband(driver.getLeftX() * Constants.Swerve.slowRegulator, Constants.Swerve.axisDeadBand));
      if(Constants.SuperStructure.isAuto && (m_vision.getID() == 4 || m_vision.getID() == 7 || m_vision.getID() == 3 || m_vision.getID() == 8)) {
          rotationVal = s_Swerve.calculateAutoFacing();
      } else {
          rotationVal =
              rotationLimiter.calculate(
                  MathUtil.applyDeadband(driver.getRightX() * Math.pow(Constants.Swerve.slowRegulator, 2), Constants.Swerve.axisDeadBand));
      }
    } else {
      translationVal =
          translationLimiter.calculate(
              MathUtil.applyDeadband(driver.getLeftY(), Constants.Swerve.axisDeadBand));
      strafeVal =
          strafeLimiter.calculate(
              MathUtil.applyDeadband(driver.getLeftX(), Constants.Swerve.axisDeadBand));
      if(Constants.SuperStructure.isAuto && (m_vision.getID() == 4 || m_vision.getID() == 7 || m_vision.getID() == 3 || m_vision.getID() == 8)){
          rotationVal = s_Swerve.calculateAutoFacing();
      } else {
          rotationVal =
              rotationLimiter.calculate(
                  MathUtil.applyDeadband(driver.getRightX() * Constants.Swerve.slowRegulator, Constants.Swerve.axisDeadBand));
      }
    }
    
    if (driver.getLeftBumperPressed() && onePress1==false) {
      Constants.Swerve.fieldOriented = !Constants.Swerve.fieldOriented;
      onePress1 = true;
    }else if(driver.getLeftBumperReleased()) {
      onePress1 = false;
    }

    if (driver.getRightBumperPressed() && onePress2==false) {
      Constants.Swerve.slow = !Constants.Swerve.slow;
      onePress2 = true;
    }else if(driver.getRightBumperReleased()) {
      onePress2 = false;
    }

    if (driver.getBackButton()) s_Swerve.zeroGyro();
    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity, Constants.Swerve.fieldOriented,
        true);

    //testing motors
    // if(driver.getPOV() == 0) s_Swerve.mSwerveMods[0].setDriveMotor(1);
    // if(driver.getPOV() == 45) s_Swerve.mSwerveMods[0].setAngleMotor(0.3);
    // if(driver.getPOV() == 90) s_Swerve.mSwerveMods[1].setDriveMotor(1);
    // if(driver.getPOV() == 135) s_Swerve.mSwerveMods[1].setAngleMotor(0.3);
    // if(driver.getPOV() == 180) s_Swerve.mSwerveMods[2].setDriveMotor(1);
    // if(driver.getPOV() == 225) s_Swerve.mSwerveMods[2].setAngleMotor(0.3);
    // if(driver.getPOV() == 270) s_Swerve.mSwerveMods[3].setDriveMotor(1);
    // if(driver.getPOV() == 315) s_Swerve.mSwerveMods[3].setAngleMotor(0.3);

  }
}