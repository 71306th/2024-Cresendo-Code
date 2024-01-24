// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.commands.TeleopSuperStructure;
import frc.robot.commands.TeleopSwerve;

public class RobotContainer {

  private final Swerve m_swerve = new Swerve();
  private final SuperStructure m_superstructure = new SuperStructure();
  private final Vision m_vision = new Vision();
  private final Controller m_controller = new Controller();

  private final TeleopSwerve teleSwerve = new TeleopSwerve(m_vision, m_swerve, m_controller);
  private final TeleopSuperStructure teleSuperStructure = new TeleopSuperStructure(m_vision, m_superstructure, m_controller);

  public RobotContainer() {
    configureBindings();

    m_swerve.setDefaultCommand(teleSwerve);
    m_superstructure.setDefaultCommand(teleSuperStructure);
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return null;
  }
}
