// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveDistance extends CommandBase {

  Drivetrain drivetrainSubsystem;
  double targetDistance;
  double driveSpeed;
  double startDistance;
  
  public DriveDistance(Drivetrain drivetrain, double distance, double driveSpeed) {
    drivetrainSubsystem = drivetrain;
    targetDistance = distance;
    this.driveSpeed = driveSpeed;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startDistance = averageDistance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrainSubsystem.arcadeDrive(driveSpeed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (averageDistance() >= startDistance + targetDistance);
  }

  // Helper method
  private double averageDistance() {
    return (drivetrainSubsystem.getLeftEncoderDistance() + drivetrainSubsystem.getRightEncoderDistance()) / 2;
  }
}
