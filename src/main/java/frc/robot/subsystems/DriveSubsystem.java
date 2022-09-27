// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.PiCamera.Logger;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  Object m_setLock = new Object();

  CANSparkMax m_leftDrive = new CANSparkMax(Constants.k_leftDrive, MotorType.kBrushless);
  CANSparkMax m_rightDrive = new CANSparkMax(Constants.k_rightDrive, MotorType.kBrushless);
  CANSparkMax m_leftFollower = new CANSparkMax(Constants.k_leftFollower, MotorType.kBrushless);
  CANSparkMax m_rightFollower = new CANSparkMax(Constants.k_rightFollower, MotorType.kBrushless);

  MotorControllerGroup m_left = new MotorControllerGroup(m_leftDrive, m_leftFollower);
  MotorControllerGroup m_right = new MotorControllerGroup(m_rightDrive, m_rightFollower);

  DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_right.setInverted(true);
    m_left.setInverted(true);
  }
  

  public void setPower(double leftPower, double rightPower){
    synchronized(m_setLock){
      m_leftDrive.set(leftPower);
      m_rightDrive.set(rightPower);
    }
  }

  public DifferentialDrive getDrive(){
    return m_drive;
  }
  
  public void setBrakeMode(boolean brake) {
    m_leftDrive.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    m_leftFollower.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    m_rightDrive.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    m_rightFollower.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    SmartDashboard.putBoolean("brake mode", brake);
    Logger.Log("DriveSubsystem", 1, "Brake  " + brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
