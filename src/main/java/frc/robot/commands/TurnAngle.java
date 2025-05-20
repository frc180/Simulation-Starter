// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class TurnAngle extends Command {

  Drivetrain drivetrainSubsystem;
  double targetAngle;
  
  public TurnAngle(Drivetrain drivetrain, double angleDegrees) {
    drivetrainSubsystem = drivetrain;
    targetAngle = Units.degreesToRadians(angleDegrees);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d gyro = drivetrainSubsystem.getGyro();
    double pidSpeed = drivetrainSubsystem.turnPid.calculate(gyro.getRadians(), targetAngle);
    drivetrainSubsystem.arcadeDrive(0, -pidSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Rotation2d gyro = drivetrainSubsystem.getGyro();
    double error = Math.abs(gyro.getRadians() - targetAngle);
    return error <= 0.05235988 / 2;
  }
}
