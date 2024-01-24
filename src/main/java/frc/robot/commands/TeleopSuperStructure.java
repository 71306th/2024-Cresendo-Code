package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.InputStates;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSuperStructure extends CommandBase {

  private final Vision m_vision;
  private final SuperStructure m_superstructure;
  private final Controller m_controller;

  private XboxController driver;
  private XboxController operator;

  private int counter;

  public TeleopSuperStructure(Vision m_vision, SuperStructure m_superstructure, Controller m_controller) {
    this.m_vision = m_vision;
    addRequirements(m_vision);
    this.m_superstructure = m_superstructure;
    addRequirements(m_superstructure);
    this.m_controller = m_controller;
    addRequirements(m_controller);
    driver = m_controller.getDriverController();
    operator = m_controller.getOperatorController();
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(operator.getAButton()) {
      if(Constants.SuperStructure.isAuto) {
        Constants.SuperStructure.isAuto = !Constants.SuperStructure.isAuto;
        m_superstructure.setState(InputStates.idle);
      } else {
        Constants.SuperStructure.isAuto = !Constants.SuperStructure.isAuto;
        m_superstructure.setState(InputStates.A);
      }
    }
    if(operator.getXButton()) {
      if(Constants.SuperStructure.isFloor) {
        Constants.SuperStructure.isFloor = !Constants.SuperStructure.isFloor;
        m_superstructure.setState(InputStates.idle);
      } else {
        Constants.SuperStructure.isFloor = !Constants.SuperStructure.isFloor;
        m_superstructure.setState(InputStates.X);
      }
    }
    if(operator.getBButton()) m_superstructure.setState(InputStates.B);
    if(operator.getYButton()) m_superstructure.setState(InputStates.Y);
    if(Math.abs(driver.getLeftY())>=0.2 || Math.abs(driver.getLeftX())>=0.2 || Math.abs(driver.getRightX())>=0.2) {
      if(!Constants.SuperStructure.isAuto && !Constants.SuperStructure.isFloor) m_superstructure.setState(InputStates.idle);
    }

    if(operator.getLeftBumper()) m_superstructure.setFIntakeSpeed(Constants.SuperStructure.FloorPushNote);
    if(operator.getRightBumper()) {
      switch(counter) {
        case 0:
          m_vision.setLEDMode(counter);
          counter++;
          break;
        case 1:
          m_vision.setLEDMode(counter);
          counter++;
          break;
        case 2:
          m_vision.setLEDMode(counter);
          counter++;
          break;
        case 3:
          m_vision.setLEDMode(counter);
          counter = 3;
          break;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
