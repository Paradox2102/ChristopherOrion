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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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

  public DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
  public static final double k_trackWidthMeters = .6;
  public final DifferentialDriveKinematics k_DriveKinematics = new DifferentialDriveKinematics(k_trackWidthMeters);

  public final double k_s = .20706;
  public final double k_v = 2.7656;
  public final double k_a = .45747; 
  public final double k_p = 1.6005E-06;
  private final double k_maxSpeed = 2.5;
  private final double k_maxAccel = 15;
  private final double k_distancePerTick = .04555;


  public final double k_maxSpeedMetersPerSecond = .25;
  public final double k_maxAccelerationMetersPerSecondSquared = 1;

  public SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(k_s, k_v, k_a);

  DifferentialDriveOdometry m_odometry;
  private final Field2d m_field = new Field2d();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_right.setInverted(true);
    m_left.setInverted(true);
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    SmartDashboard.putData("Get Pose Meters", m_field);
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

  public double getHeading(){
    return m_gyro.getRotation2d().getDegrees();
  }

  public void setSpeed(double leftSpeed, double rightSpeed){
    m_leftDrive.setVoltage(m_feedForward.calculate(leftSpeed));
    m_rightDrive.setVoltage(m_feedForward.calculate(rightSpeed));
  }

  public DifferentialDrive getDrive(){
    return m_drive;
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }
  
  public void setBrakeMode(boolean brake) {
    m_leftDrive.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    m_leftFollower.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    m_rightDrive.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    m_rightFollower.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    SmartDashboard.putBoolean("brake mode", brake);
    Logger.Log("DriveSubsystem", 1, "Brake  " + brake);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity() * k_distancePerTick, m_rightEncoder.getVelocity() * k_distancePerTick);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    m_left.setVoltage(leftVolts);
    m_right.setVoltage(rightVolts);
    m_drive.feed();
  }

  @Override
  public void periodic() {
    // This method will bme called once per scheduler run
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getPosition() * k_distancePerTick, m_rightEncoder.getPosition() * k_distancePerTick);
    m_field.setRobotPose(m_odometry.getPoseMeters());
    SmartDashboard.putNumber("Right Encoder", m_rightEncoder.getPosition());
    SmartDashboard.putNumber("Left Encoder", m_leftEncoder.getPosition());
  }

}
