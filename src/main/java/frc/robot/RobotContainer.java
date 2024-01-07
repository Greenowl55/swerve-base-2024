// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.TeleOpCommand;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class RobotContainer {
  public static final Joystick m_driverL = new Joystick(0);
  static final Joystick m_driverR = new Joystick(1);
  public static final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();

  public RobotContainer() {
    m_drivetrain.setDefaultCommand(new TeleOpCommand(m_driverL::getX, m_driverL::getY, m_driverR::getX, m_drivetrain));
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}