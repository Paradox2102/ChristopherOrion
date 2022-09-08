// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ThroatSubsystem extends SubsystemBase {
  private TalonSRX m_throatMotor = new TalonSRX(Constants.k_throat);

  private DigitalInput m_dioTop = new DigitalInput(Constants.k_dioTop);
  private DigitalInput m_dioBottom = new DigitalInput(Constants.k_dioBottom);

  private DigitalInput m_test = new DigitalInput(4);

  /** Creates a new ThroatSubsystem. */
  public ThroatSubsystem() {
    m_throatMotor.configContinuousCurrentLimit(30);
  }

  public void setPower(double power){
    m_throatMotor.set(ControlMode.PercentOutput, power);
  }

  public boolean getTop(){
    return !m_dioTop.get();
  }

  public boolean getBottom(){
    return !m_dioBottom.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Linebreak Top Sensor", m_dioTop.get());
    SmartDashboard.putBoolean("Linebreak Bottom Sensor", m_dioBottom.get());
    SmartDashboard.putBoolean("Line break test", m_test.get());
  }
}
