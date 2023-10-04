// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TapeTurnerSubsystem extends SubsystemBase {

  private final CANSparkMax m_motor;
//  private final AbsoluteEncoder m_encoder;

  /** Creates a new ExampleSubsystem. */
  public TapeTurnerSubsystem() {
    m_motor = new CANSparkMax(Constants.OperatorConstants.kTapeTurnerMotor, MotorType.kBrushless);
  //  m_encoder = m_motor.getAbsoluteEncoder(null);
  }

  public void turn() {
    m_motor.set(-0.25);
  }

  public void stop() {
    m_motor.set(0);
  }
}
