// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project. 

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private final CANSparkMax m_shooter = new CANSparkMax(Constants.k_shooter, MotorType.kBrushless);
  private final CANSparkMax m_shooterFollower = new CANSparkMax(Constants.k_shooterFollower, MotorType.kBrushless);
  private final CANSparkMax m_backWheels = new CANSparkMax(Constants.k_backWheels, MotorType.kBrushless);
  private final SparkMaxPIDController m_shooterController = m_shooter.getPIDController();
  private final SparkMaxPIDController m_backWheelController = m_backWheels.getPIDController();
  private final RelativeEncoder m_shooterEncoder =  m_shooter.getEncoder();
  private final RelativeEncoder m_backWheelEncoder = m_backWheels.getEncoder();

  private final double k_maxFrontShooterSpeed = 3800;
  private final double k_frontF = .8435/k_maxFrontShooterSpeed;
  private final double k_frontP = .0022;
  private final double k_frontI = .0000005;
  private final double k_frontD = 0;
  private final double k_frontIZone = 200;

  private final double k_maxBackShooterSpeed = 4200;
  private final double k_backF = .9066/k_maxBackShooterSpeed;
  private final double k_backP = .00015;
  private final double k_backI = .0000005;
  private final double k_backD = 0;
  private final double k_backIZone = 200;


  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_shooter.restoreFactoryDefaults();
    m_shooterFollower.restoreFactoryDefaults();
    m_shooterFollower.follow(m_shooter, true);
    m_shooter.setInverted(false);

    m_shooterController.setFF(k_frontF);
    m_shooterController.setP(k_frontP);
    m_shooterController.setI(k_frontI);
    m_shooterController.setD(k_frontD);
    m_shooterController.setIZone(k_frontIZone);

    m_backWheelController.setFF(k_backF);
    m_backWheelController.setP(k_backP);
    m_backWheelController.setI(k_backI);
    m_backWheelController.setD(k_backD);
    m_backWheelController.setIZone(k_backIZone);

    // m_shooter.setSelectedSensorPosition(0);
    // m_shooter.setSensorPhase(true);
  }

  public void setPower(double frontPower, double backPower){
    m_shooter.set(frontPower);
    m_backWheels.set(backPower);
  }

  public void setSpeed(double frontSpeed, double backSpeed){
    m_shooterController.setReference(frontSpeed, CANSparkMax.ControlType.kVelocity);
    m_backWheelController.setReference(backSpeed, CANSparkMax.ControlType.kVelocity);
  }

  public double getFrontSpeed(){
    return m_shooterEncoder.getVelocity();
  }

  public double getBackSpeed(){
    return m_backWheelEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Speed", getFrontSpeed());
    SmartDashboard.putNumber("Back Wheel Speed", getBackSpeed());
    SmartDashboard.putString("hiii", "hello");
  }
}
