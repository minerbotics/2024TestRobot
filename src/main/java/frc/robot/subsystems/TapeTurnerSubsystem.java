// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TapeTurnerSubsystem extends SubsystemBase {

  private final CANSparkMax m_motor;
  private final RelativeEncoder m_encoder;

  /** Creates a new ExampleSubsystem. */
  public TapeTurnerSubsystem() {
    m_motor = new CANSparkMax(Constants.OperatorConstants.kTapeTurnerMotor, MotorType.kBrushless);
    m_encoder = m_motor.getAlternateEncoder(Type.kQuadrature, 4096);
  }

  public void turn(boolean clockwise) {
    double speed = 0.0625;
    if (clockwise) {
      m_motor.set(-speed);
    } else {
      m_motor.set(speed);
    }
    SmartDashboard.putNumber("TapeTurner", m_encoder.getPosition());
  }

  public void stop() {
    m_motor.set(0);
  }
}
