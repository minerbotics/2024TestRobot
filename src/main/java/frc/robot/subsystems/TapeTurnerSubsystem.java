// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TapeTurnerSubsystem extends SubsystemBase {

  private final CANSparkMax m_motor;
  private final RelativeEncoder m_encoder;
  private final SparkMaxPIDController m_pidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  /** Creates a new ExampleSubsystem. */
  public TapeTurnerSubsystem() {
    m_motor = new CANSparkMax(Constants.OperatorConstants.kTapeTurnerMotor, MotorType.kBrushless);
    m_pidController = m_motor.getPIDController();
    m_encoder = m_motor.getAlternateEncoder(Type.kQuadrature, 4096);
    // PID coefficients
    kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
  }

  public void turn(boolean clockwise) {
    double speed = 0.0625;
    if (clockwise) {
      m_motor.set(-speed);
    } else {
      m_motor.set(speed);
    }
    SmartDashboard.putNumber("TapeTurner Position", m_encoder.getPosition());
    SmartDashboard.putNumber("TapeTurner Velocity", m_encoder.getVelocity());
  }

  public void turnToPosition(double position) {
    m_pidController.setReference(position, ControlType.kPosition);
  }

  public void stop() {
    m_motor.set(0);
  }
}
