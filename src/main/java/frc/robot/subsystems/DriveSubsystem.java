// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
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

  RelativeEncoder m_leftEncoder = m_leftDrive.getEncoder();
  RelativeEncoder m_rightEncoder = m_rightDrive.getEncoder();

  Gyro m_gyro = new WPI_PigeonIMU(0);

  MotorControllerGroup m_left = new MotorControllerGroup(m_leftDrive, m_leftFollower);
  MotorControllerGroup m_right = new MotorControllerGroup(m_rightDrive, m_rightFollower);

  DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
  public static final double k_trackWidthMeters = .6;
  public static final DifferentialDriveKinematics k_DriveKinematics = new DifferentialDriveKinematics(k_trackWidthMeters);

  private final double k_s = .20706;
  private final double k_v = 2.7656;
  private final double k_a = .45747; 
  private final double k_p = 1.6005E-06;
  private final double k_maxSpeed = 2.5;
  private final double k_maxAccel = 15;
  private final double k_distancePerTick = .4555;

  SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(k_s, k_v, k_a);

  DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_right.setInverted(true);
    m_left.setInverted(true);
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }
  

  public void setPower(double leftPower, double rightPower){
    synchronized(m_setLock){
      m_leftDrive.set(leftPower);
      m_rightDrive.set(rightPower);
    }
  }

  public void resetEncoders(){
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public void setSpeed(double leftSpeed, double rightSpeed){
    m_leftDrive.setVoltage(m_feedForward.calculate(leftSpeed));
    m_rightDrive.setVoltage(m_feedForward.calculate(rightSpeed));
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
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getPosition() * k_distancePerTick, m_rightEncoder.getPosition() * k_distancePerTick);
  }

}
