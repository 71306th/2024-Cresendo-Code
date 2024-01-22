// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ChenryLib.MathUtility;
import frc.ChenryLib.PID;
import frc.robot.Constants;

public class SuperStructure extends SubsystemBase {

  private Vision m_vision;

  private final XboxController driver;
  private final XboxController operator;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  
  private final WPI_TalonFX tilter;
  private final CANSparkMax floor;
  private final CANSparkMax uppermaster;
  private final CANSparkMax upperslave;

  private final PID tilterPID;
  private final PID flywheelPID;

  private final DigitalInput limitSwitch;

  public static enum TilterStates {
    auto,
    podium,
    floor,
    human,
    idle
  }

  public static enum IntakeStates {
    auto,
    podium,
    floor,
    human,
    idle
  }

  public static enum InputStates {
    A,
    B,
    X,
    Y,
    idle
  }

  private double tilterCurrentAngle;
  private double tilterGoalAngle;
  private double intakeUCurrentSpeed;
  private double intakeUGoalSpeed;

  private InputStates CommandState;
  private TilterStates TilterState;
  private IntakeStates IntakeState;

  /* Distance calculating(it's too annoying) */
  double TagToTilterx = m_vision.TagToBotCenter().getX() + Constants.SuperStructure.TilterToBotCenter.getX();
  double TagToTiltery = m_vision.TagToBotCenter().getY() + Constants.SuperStructure.TilterToBotCenter.getY();
  double TagToTilterz = m_vision.TagToBotCenter().getZ() + Constants.SuperStructure.TilterToBotCenter.getZ();

  public SuperStructure() {
    tilter = new WPI_TalonFX(Constants.SuperStructure.tilterID);
    floor = new CANSparkMax(Constants.SuperStructure.floorMotorID, MotorType.kBrushed);
    uppermaster = new CANSparkMax(Constants.SuperStructure.uppermasterMotorID, MotorType.kBrushless);
    upperslave = new CANSparkMax(Constants.SuperStructure.upperslaveMotorID, MotorType.kBrushless);
    
    upperslave.follow(uppermaster, false);

    limitSwitch = new DigitalInput(Constants.SuperStructure.limitSwitch);

    tilterPID = new PID(0, 0, 0, 0, 0);
    flywheelPID = new PID(0, 0, 0, 0, 0);

    CommandState = InputStates.idle;
    TilterState = TilterStates.idle;
    IntakeState = IntakeStates.idle;

    driver = new XboxController(Constants.JoystickConstants.kDriverControllerPort);
    operator = new XboxController(Constants.JoystickConstants.kOperatorControllerPort);
  }

  @Override
  public void periodic() {
    tilterCurrentAngle = getTilterDegrees();
    intakeUCurrentSpeed = getIntakeVelocity();
    updateOverallStates();
    updateTilterStates();
    updateIntakeStates();
    if(m_vision.getID()==4 || m_vision.getID() == 7 || m_vision.getID() == 3 || m_vision.getID() == 8) {
      tilterGoalAngle = calculatingTilterAngle();
      intakeUGoalSpeed = calculatingShootingVelocity();
    }
    SmartDashboard.putString("Input State", CommandState.toString());
    SmartDashboard.putNumber("Tilter Current Angle", tilterCurrentAngle);
    SmartDashboard.putNumber("Intake Current Speed", intakeUCurrentSpeed);
  }

  /* EZ settings */
  public void setUIntakeSpeed(double speed) {
    uppermaster.set(speed);;
  }

  public void setFIntakeSpeed(double speed) {
    floor.set(speed);
  }

  public void setTilterSpeed(double speed) {
    tilter.set(speed);
  }

  public void setState(InputStates state) {
    CommandState = state;
  }


  /* Head-Exploding fetches */
  public double getTilterDegrees() {
    return tilter.getSelectedSensorPosition() * 360 / Constants.SuperStructure.falconEncooderCounts;
  }

  public double getIntakeVelocity() {
    return (uppermaster.getEncoder().getVelocity() + upperslave.getEncoder().getVelocity()) / 2;
  }

  public InputStates getState() {
    return CommandState;
  }

  public double getSpeakerToUIntake() {
    if(m_vision.getID() == 4 || m_vision.getID() == 7){
      return Math.sqrt(
        Math.pow(TagToTilterx + (Constants.SuperStructure.IntakeMToTilter / Math.cos(Constants.SuperStructure.IntakeUToTilterAngle)) * Math.cos(tilterCurrentAngle), 2) +
        Math.pow(TagToTiltery, 2) +
        Math.pow(TagToTilterz + (Constants.SuperStructure.IntakeMToTilter / Math.cos(Constants.SuperStructure.IntakeUToTilterAngle)) * Math.sin(tilterCurrentAngle) + Constants.Vision.SpeakerIDToSpeakerZ, 2)
      ) / Math.cos(m_vision.TagToBotCenter().getRotation().getAngle() * Math.PI / 180);
    }else if(m_vision.getID() == 3 || m_vision.getID() == 8){
      return Math.sqrt(
        Math.pow(TagToTilterx + (Constants.SuperStructure.IntakeMToTilter / Math.cos(Constants.SuperStructure.IntakeUToTilterAngle)) * Math.cos(tilterCurrentAngle), 2)+
        Math.pow(TagToTiltery + Constants.Vision.SpeakerIDToSpeakerY, 2)+
        Math.pow(TagToTilterz + (Constants.SuperStructure.IntakeMToTilter / Math.cos(Constants.SuperStructure.IntakeUToTilterAngle)) * Math.sin(tilterCurrentAngle) + Constants.Vision.SpeakerIDToSpeakerZ, 2)
      ) / Math.cos(m_vision.TagToBotCenter().getRotation().getAngle() * Math.PI / 180);
    }else return 0;
  } // need review, not sure it's right or not

  /* OH GOD PLEASE NO MY BRAIN'S DYING calculations */
  public double calculatingTilterAngle() {
    double theta = Math.acos(
      (Math.sqrt(Math.pow(TagToTilterx, 2) + Math.pow(TagToTiltery, 2))) / TagToTilterz);
    double phi = Math.acos(
      (Constants.SuperStructure.IntakeMToTilter) / 
      Math.sqrt((Math.pow(TagToTilterx, 2) + Math.pow(TagToTiltery, 2) + Math.pow(TagToTilterz, 2))));
    double output = 180-theta-phi;
    if(90-output < Constants.SuperStructure.restrictedLowShootingAngle) return -1;
    else return (output);
  } // need review, not sure it's right or not

  public double calculatingShootingVelocity() {
    double g = 9.80665;
    double x = Math.sqrt(Math.pow(TagToTilterx, 2) + Math.pow(TagToTiltery, 2));
    double y = TagToTilterz;
    double theta = 90 - calculatingTilterAngle();
    double robotXVelocity = Constants.Swerve.slow ? strafeLimiter.calculate(MathUtil.applyDeadband(driver.getLeftX() * Constants.Swerve.slowRegulator, Constants.Swerve.axisDeadBand)) : strafeLimiter.calculate(MathUtil.applyDeadband(driver.getLeftX(), Constants.Swerve.axisDeadBand));
    double robotYVelocity = Constants.Swerve.slow ? strafeLimiter.calculate(MathUtil.applyDeadband(driver.getLeftY() * Constants.Swerve.slowRegulator, Constants.Swerve.axisDeadBand)) : strafeLimiter.calculate(MathUtil.applyDeadband(driver.getLeftY(), Constants.Swerve.axisDeadBand));
    double formulaInitialSpeed = Math.sqrt(-g*Math.pow(x, 2) / ((2 * Math.cos(theta) * Math.sin(theta) * x) - (x * Math.pow(Math.cos(theta), 2) * y))); // trajectory formula of diagonal throw
    double vectorXofv0 = formulaInitialSpeed * Math.cos(theta);
    double vectoryofv0 = formulaInitialSpeed * TagToTilterx / Math.sqrt(Math.pow(TagToTilterx, 2) + Math.pow(TagToTiltery, 2));
    double vectorzofv0 = formulaInitialSpeed * Math.cos(90 - theta);
    double output = Math.sqrt(Math.pow(vectorXofv0 - robotXVelocity, 2) + Math.pow(vectoryofv0 - robotYVelocity, 2) + Math.pow(vectorzofv0, 2));
    if(output > Constants.SuperStructure.restrictedMaxMotorVelocity) return -1;
    else return output;
  } // need review, not sure it's right or not


  /* Goddamn state machines */
  public void updateOverallStates() {
    switch(CommandState) {
      case A:
        TilterState = TilterStates.auto;
        IntakeState = IntakeStates.auto;
        operator.setRumble(RumbleType.kBothRumble, 1);
        break;
      case B:
        TilterState = TilterStates.human;
        IntakeState = IntakeStates.human;
        operator.setRumble(RumbleType.kBothRumble, 1);
        break;
      case X:
        TilterState = TilterStates.floor;
        IntakeState = IntakeStates.floor;
        operator.setRumble(RumbleType.kBothRumble, 1);
        break;
      case Y:
        TilterState = TilterStates.podium;
        IntakeState = IntakeStates.podium;
        operator.setRumble(RumbleType.kBothRumble, 1);
        break;
      case idle:
        TilterState = TilterStates.idle;
        IntakeState = IntakeStates.idle;
        break;
    }
  }

  public void updateTilterStates() {
    switch(TilterState) {
      case auto:
        setTilterSpeed(MathUtility.clamp(tilterPID.calculate(tilterGoalAngle - tilterCurrentAngle), -0.3, 0.3));
        break;
      case human:
        setTilterSpeed(MathUtility.clamp(tilterPID.calculate(Constants.SuperStructure.TilterHumanIntake - tilterCurrentAngle), -0.3, 0.3));
        break;
      case floor:
        setTilterSpeed(MathUtility.clamp(tilterPID.calculate(Constants.SuperStructure.TilterFloorIntake - tilterCurrentAngle), -0.3, 0.3));
        break;
        case podium:
        setTilterSpeed(MathUtility.clamp(tilterPID.calculate(Constants.SuperStructure.TilterPodiumToSpeaker - tilterCurrentAngle), -0.3, 0.3));
        break;
      case idle:
        setTilterSpeed(MathUtility.clamp(tilterPID.calculate(Constants.SuperStructure.TilterIdle), -0.3, 0.3));
        break;
    }
  }

  public void updateIntakeStates() {
    switch(IntakeState) {
      case auto:
        if(intakeUGoalSpeed != -1) setUIntakeSpeed(MathUtility.clamp(flywheelPID.calculate(intakeUGoalSpeed - intakeUCurrentSpeed), -1, 1));
        else setUIntakeSpeed(MathUtility.clamp(flywheelPID.calculate(Constants.SuperStructure.IntakeIdle - intakeUCurrentSpeed), -1, 1));
        break;
      case human:
        setUIntakeSpeed(MathUtility.clamp(flywheelPID.calculate(Constants.SuperStructure.HumanIntake - intakeUCurrentSpeed), -1, 1));
        break;
      case floor:
        setFIntakeSpeed(MathUtility.clamp(flywheelPID.calculate(Constants.SuperStructure.FloorIntake - intakeUCurrentSpeed), -1, 1));
        break;
      case podium:
        setUIntakeSpeed(MathUtility.clamp(flywheelPID.calculate(Constants.SuperStructure.PodiumToSpeaker - intakeUCurrentSpeed), -1, 1));
        break;
      case idle:
        setUIntakeSpeed(MathUtility.clamp(flywheelPID.calculate(Constants.SuperStructure.IntakeIdle - intakeUCurrentSpeed), -1, 1));
        setFIntakeSpeed(MathUtility.clamp(flywheelPID.calculate(Constants.SuperStructure.IntakeIdle - intakeUCurrentSpeed), -1, 1));
        break;
    }
  }
}
