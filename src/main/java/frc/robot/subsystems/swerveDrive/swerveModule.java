// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerveDrive;

import static frc.robot.constants.driveConstants.swerveConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.fasterxml.jackson.core.StreamReadCapability;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



import java.lang.Math;

import javax.naming.ldap.LdapName;




public class SwerveModule extends SubsystemBase {
  private final TalonFX driveMotor;
  private final CANSparkMax angleMotor;
  private final CANCoder absoluteEncoder;

  private final RelativeEncoder steerEncoder;

  private final SparkMaxPIDController angleController;

  private final double rotationOffset;
  private final double rotationOffsetRad;

  private final AHRS gyro;

  private final int id;
  private SwerveModuleState lastState;
  

  public SwerveModule(int driveID, int steerID, int absoluteID, double rotationOffset, AHRS gyro, int id) {
    driveMotor = new TalonFX(driveID);
    angleMotor = new CANSparkMax(steerID, MotorType.kBrushless);
    absoluteEncoder = new CANCoder(absoluteID);
    steerEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();

    this.rotationOffset = rotationOffset;
    this.rotationOffsetRad = (rotationOffset / 360) * (Math.PI * 2);

    this.gyro = gyro;
    this.id = id;
    
    driveMotor.setNeutralMode(NeutralMode.Brake);
    configAngleMotor();

    // converts position and velocity to radians and radians per sec respectivily
  }

  public double getAbsoluteEncoder() {
            
      return (absoluteEncoder.getAbsolutePosition());

  }
  public double getAbsoluteEncoderRad() {
      return (absoluteEncoder.getAbsolutePosition() / 360) * (Math.PI * 2);

  }


  public void updateSteerEncoder(){
    var value = (rotationOffset - getAbsoluteEncoder()) % 360;

    if (value > 180){
      value = value - 360;
    }

    System.out.print(String.valueOf(id) + ": " + String.valueOf(value));
     
     SmartDashboard.putNumber("Positon of Actual Val " + String.valueOf(id)+": ", value);
     steerEncoder.setPosition(value);
  }


  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    angleMotor.setInverted(false);
    angleMotor.setIdleMode(IdleMode.kCoast);
    steerEncoder.setPositionConversionFactor(360/
    steerGearRatio);
    
    angleController.setPositionPIDWrappingMinInput(-180);
    angleController.setPositionPIDWrappingMaxInput(180);
    angleController.setPositionPIDWrappingEnabled(true);

    angleController.setP(0.01);
    angleController.setI(0);
    angleController.setD(0);
    angleController.setFF(0);


    updateSteerEncoder();
  }

  

 

  public void setAngle(double value){
      SmartDashboard.putNumber("Angle Final" + String.valueOf(id), value);
   
      angleMotor.getPIDController().setReference(value, ControlType.kPosition);
   
  }

  public void setDrive(double value){
    SmartDashboard.putNumber(String.valueOf(id), value);
    driveMotor.set(ControlMode.PercentOutput, value * 0.2);
  }
  public SwerveModuleState getState(){
    return new SwerveModuleState(0 , new Rotation2d((getAbsoluteEncoderRad() - rotationOffsetRad) % (Math.PI * 2)));
    
  }

  public void setState(SwerveModuleState state){
    state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(steerEncoder.getPosition()));
      if (Math.abs(state.speedMetersPerSecond) < 0.1){
        angleMotor.set(0);
        setDrive(0);
      }
      
      else{
        setAngle(state.angle.getDegrees());
        setDrive(state.speedMetersPerSecond);
      }
    }
  
    

  @Override
  public void periodic() {
   

   
    // System.out.println(String.valueOf(id) + " - Absolute Encoder:   Deg:  " + String.valueOf(getAbsoluteEncoder()) + " --- Rad:  " + String.valueOf(getAbsoluteEncoderRad()));
  }
}
