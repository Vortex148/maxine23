// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.ModuleLayer.Controller;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.operatorDriveCommand;
import frc.robot.subsystems.swerveDrive.SwerveSubsystem;
import frc.robot.subsystems.swerveDrive.SwerveSubsystem.*;

public class RobotContainer {
  private static XboxController controller = new XboxController(0);
  private static SwerveSubsystem drivetrain = new SwerveSubsystem();
  
  public Command getDriveCommand() {
    return new operatorDriveCommand(
      drivetrain,
      controller
    );
}

  public RobotContainer() {
    drivetrain.setDefaultCommand(getDriveCommand());
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
