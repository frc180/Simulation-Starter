// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveDistancePID extends Command {

  Drivetrain drivetrainSubsystem;
  double targetDistance;
  double startDistance;
  
  public DriveDistancePID(Drivetrain drivetrain, double distance) {
    drivetrainSubsystem = drivetrain;
    targetDistance = distance;
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
    double pidSpeed = drivetrainSubsystem.drivePid.calculate(averageDistance(), startDistance + targetDistance);
    drivetrainSubsystem.arcadeDrive(pidSpeed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double error = Math.abs(averageDistance() - (startDistance + targetDistance));
    return error <= 0.0025;
  }

  // Helper method
  private double averageDistance() {
    return (drivetrainSubsystem.getLeftEncoderDistance() + drivetrainSubsystem.getRightEncoderDistance()) / 2;
  }
}
