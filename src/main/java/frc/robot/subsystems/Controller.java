// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Controller extends SubsystemBase {
  
  public final XboxController driver;
  public final XboxController operator;

  public Controller() {
    driver = new XboxController(Constants.JoystickConstants.kDriverControllerPort);
    operator = new XboxController(Constants.JoystickConstants.kOperatorControllerPort);
  }

  @Override
  public void periodic() {}

  public XboxController getDriverController() {
    return driver;
  }

  public XboxController getOperatorController() {
    return operator;
  }
}
