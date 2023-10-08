// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerveDrive;

import static frc.robot.constants.driveConstants.swerveConstants.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.driveConstants;
import frc.robot.subsystems.swerveDrive.swerveModule;

public class swerveSubsystem extends SubsystemBase {

  private final swerveModule frontLeft = new swerveModule(driveLeftFront, steerLeftFront, encoderLeftFront);
  private final swerveModule frontRight = new swerveModule(driveRightFront, steerRightFront, encoderRightFront);
  private final swerveModule backLeft = new swerveModule(driveLeftBack, steerLeftBack, encoderLeftBack);
  private final swerveModule backRight = new swerveModule(driveRightBack, steerRightBack, encoderRightBack);

  private final AHRS gyro  = new AHRS(SPI.Port.kMXP);

  private void zeroGyro(){
    gyro.reset();
  }
  /** Creates a new swerveSubsystem. */
  public swerveSubsystem() {
    zeroGyro();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
