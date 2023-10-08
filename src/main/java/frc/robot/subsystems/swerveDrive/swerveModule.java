// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerveDrive;

import static frc.robot.constants.driveConstants.swerveConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.opencv.core.Mat;

import java.lang.Math;

public class swerveModule extends SubsystemBase {

  private final TalonFX driveMotor;
  private final CANSparkMax steerMotor;
  private final CANCoder absoluteEncoder;

  private final RelativeEncoder steerEncoder;

  private final PIDController steeringPID;

  public swerveModule(int driveID, int steerID, int absoluteID) {
    driveMotor = new TalonFX(driveID);
    steerMotor = new CANSparkMax(steerID, MotorType.kBrushless);
    absoluteEncoder = new CANCoder(absoluteID);
    steerEncoder = steerMotor.getEncoder();

    steeringPID = new PIDController(0.0, 0, 0);
    steeringPID.enableContinuousInput(-Math.PI, Math.PI);

    // converts position and velocity to radians and radians per sec respectivily
    steerEncoder.setPositionConversionFactor(steerGearRatio * 2 * Math.PI);
    steerEncoder.setVelocityConversionFactor((steerGearRatio * 2 * Math.PI) / 60);
  }

  public double getAbsoluteEncoder(){
    return absoluteEncoder.getAbsolutePosition();
  }
  // Should work I think from what I can find online about velocity mode
  public void setDriveSpeedMperS(double speed){
    driveMotor.set(ControlMode.Velocity, speed  * (10.0 / 2048.0) * wheelDiameter);
  }


  // created convertToRadians function in case future use is needed
  private double convertToRadians(double value, double valueMax){
    return (value / valueMax) * (2 * Math.PI);
  }

  private double convertToRadians(double value, double valueMax, double gearRatio){
    return ((value / valueMax) / gearRatio) * (2 * Math.PI);
  }

  private double convertToDegrees(double value, double valueMax){
    return (value / valueMax) * value;
  }

  private double convertToDegrees(double value, double valueMax, double gearRatio){
    return ((value / valueMax) / gearRatio) * value;
  }

  public double getDriveMotorReadingRad(){
    return convertToRadians(driveMotor.getSelectedSensorPosition(), 2048.0, driveGearRatio);
    // 2048 is the total resolution of the falcon 500s encoder
  }

  public double getDriveMotorReadingDeg(){
    return convertToDegrees(driveMotor.getSelectedSensorPosition(), 2048.0, driveGearRatio);
    // 2048 is the total resolution of the falcon 500s encoder
  }

  /**  This function is used to convert the inbuilt function which provides the sensor units over 100ms to metres to 1s
   * @return the drive velocity in m/s
  */
  public double getDriveMotorMPerS(){
    return driveMotor.getSelectedSensorVelocity() * (10.0 / 2048.0) * wheelDiameter;
  }

  public double getSteerMotorReading() {
    return steerEncoder.getPosition();
  }

  public double getAbsoluteEncoderReadingRad(){
    return convertToRadians(absoluteEncoder.getAbsolutePosition(), 4096);
    // 4096 is CANCoder resolution
  }

  public void resetEncoders(){
    driveMotor.setSelectedSensorPosition(0);
    steerEncoder.setPosition(getAbsoluteEncoder());
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveMotorMPerS(), new Rotation2d(getSteerMotorReading()));
  }

  public void setState(SwerveModuleState state){
    if (Math.abs(state.speedMetersPerSecond) > 0.001){
      // should prevent modules from returning to zero when stick is released
      setDriveSpeedMperS(0);
      steerMotor.set(0);
    }

    state = SwerveModuleState.optimize(state, getState().angle);
    setDriveSpeedMperS(state.speedMetersPerSecond);
    steerMotor.set(steeringPID.calculate(getSteerMotorReading(), state.angle.getRadians()));
  }


  @Override
  public void periodic() {
    
  }

}
