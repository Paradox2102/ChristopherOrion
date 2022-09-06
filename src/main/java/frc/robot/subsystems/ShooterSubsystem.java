// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  TalonSRX m_shooter = new TalonSRX(Constants.k_shooter);
  TalonSRX m_shooterFollower = new TalonSRX(Constants.k_shooterFollower);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_shooter.setInverted(true);
    m_shooterFollower.follow(m_shooter);
    m_shooterFollower.setInverted(false);

    m_shooter.setSelectedSensorPosition(0);
    m_shooter.setSensorPhase(true);
  }

  public void setPower(double power){
    m_shooter.set(ControlMode.PercentOutput, power);
  }

  public double getSpeed(){
    return m_shooter.getSelectedSensorVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
