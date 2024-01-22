package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.InputStates;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSuperStructure extends CommandBase {

  private final SuperStructure s_SuperStructure;

  XboxController driver = new XboxController(Constants.JoystickConstants.kDriverControllerPort);
  XboxController operator = new XboxController(Constants.JoystickConstants.kOperatorControllerPort);

  public TeleopSuperStructure(SuperStructure subsystem) {
    s_SuperStructure = subsystem;
    addRequirements(s_SuperStructure);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(operator.getAButton()) {
      if(Constants.SuperStructure.isAuto) {
        Constants.SuperStructure.isAuto = !Constants.SuperStructure.isAuto;
        s_SuperStructure.setState(InputStates.idle);
      } else {
        Constants.SuperStructure.isAuto = !Constants.SuperStructure.isAuto;
        s_SuperStructure.setState(InputStates.A);
      }
    }
    if(operator.getXButton()) {
      if(Constants.SuperStructure.isFloor) {
        Constants.SuperStructure.isFloor = !Constants.SuperStructure.isFloor;
        s_SuperStructure.setState(InputStates.idle);
      } else {
        Constants.SuperStructure.isFloor = !Constants.SuperStructure.isFloor;
        s_SuperStructure.setState(InputStates.X);
      }
    }
    if(operator.getBButton()) s_SuperStructure.setState(InputStates.B);
    if(operator.getYButton()) s_SuperStructure.setState(InputStates.Y);
    if(Math.abs(driver.getLeftY())>=0.2 || Math.abs(driver.getLeftX())>=0.2 || Math.abs(driver.getRightX())>=0.2) {
      if(!Constants.SuperStructure.isAuto && !Constants.SuperStructure.isFloor) s_SuperStructure.setState(InputStates.idle);
    }

    if(operator.getLeftBumper()) s_SuperStructure.setFIntakeSpeed(Constants.SuperStructure.FloorPushNote);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
