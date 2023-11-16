
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerveDrive;

import static frc.robot.constants.driveConstants.swerveConstants.*;

import javax.print.event.PrintJobListener;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
  private final AHRS gyro  = new AHRS(SPI.Port.kMXP);

  private final SwerveModule frontLeft = new SwerveModule(driveLeftFront, steerLeftFront, encoderLeftFront, thetaOffsetLeftFront, gyro, 1);
  private final SwerveModule frontRight = new SwerveModule(driveRightFront, steerRightFront, encoderRightFront, thetaOffsetRightFront, gyro, 2);
  private final SwerveModule backLeft = new SwerveModule(driveLeftBack, steerLeftBack, encoderLeftBack, thetaOffsetLeftBack, gyro,3);
  private final SwerveModule backRight = new SwerveModule(driveRightBack, steerRightBack, encoderRightBack, thetaOffsetRightBack, gyro,4);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(xyOffsetLeftFront, xyOffsetLeftBack, xyOffsetRightFront, xyOffsetRightBack);

  private void zeroGyro(){
    gyro.reset();
  }

  /** Creates a new swerveSubsystem. */
  public SwerveSubsystem() {
    zeroGyro();
  }

  public void operatorDrive(double x, double y, double r){
    System.out.println("X: " + x);
    System.out.println("R: " + r);
    ChassisSpeeds speeds = new ChassisSpeeds((x * 4), (y * 4), ((r * 2) /.0508));
    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(speeds); 

    SwerveModuleState frontLeftState = swerveModuleStates[0];
    SwerveModuleState frontRightState = swerveModuleStates[1];
    SwerveModuleState backLeftState = swerveModuleStates[2];
    SwerveModuleState backRightState = swerveModuleStates[3];

    frontLeft.setState(frontLeftState);
    frontRight.setState(frontRightState);
    backLeft.setState(backLeftState);
    backRight.setState(backRightState);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
