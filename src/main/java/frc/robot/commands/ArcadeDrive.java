// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDrive extends CommandBase {
  DriveSubsystem m_subsystem;
  DoubleSupplier m_getX;
  DoubleSupplier m_getY;
  DoubleSupplier m_getThrottle;
  SlewRateLimiter m_filter = new SlewRateLimiter(4);
  // 1 divided by number in perenthesis ^^^

  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(DriveSubsystem driveSubsystem, DoubleSupplier getX, DoubleSupplier getY, DoubleSupplier getThrottle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = driveSubsystem;
    m_getX = getX;
    m_getY = getY;
    m_getThrottle = getThrottle;
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("ArcadeDrive", 1, "initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = m_getX.getAsDouble();
    double y = -m_getY.getAsDouble();
    double throttle = m_getThrottle.getAsDouble();

    y = MathUtil.applyDeadband(y, .1);
    x = MathUtil.applyDeadband(x, .1);

    //m_subsystem.getDrive().arcadeDrive(y, x, true);

    if (throttle < 0){
      m_subsystem.setSpeed(y+x, y-x);//getDrive().arcadeDrive(m_filter.calculate(y), x, true);
    } else {
      m_subsystem.setSpeed(x-y, -x-y);//getDrive().arcadeDrive(m_filter.calculate(-y), x, true);
    }
    // Logger.Log("ArcadeDrive", 1, String.format("x=%f, y=%f, throttle=%f", x, y, throttle));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("ArcadeDrive", 1, "end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
