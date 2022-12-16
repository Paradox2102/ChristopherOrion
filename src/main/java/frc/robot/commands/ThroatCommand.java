// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.subsystems.ThroatSubsystem;

public class ThroatCommand extends CommandBase {
  ThroatSubsystem m_subsystem;
  private final double k_speed = -.56;

  /** Creates a new ThroatCommand. */
  public ThroatCommand(ThroatSubsystem throatSubsystem) {
    m_subsystem = throatSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("ThroatCommand", 1, "initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_subsystem.getBottom() && !m_subsystem.getTop()){
      m_subsystem.setPower(k_speed);
    } else {
      m_subsystem.setPower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setPower(0);
    Logger.log("ThroatCommand", 1, "end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
