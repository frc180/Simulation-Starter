// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveDistancePID;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

  public final Drivetrain drivetrainSubsystem = new Drivetrain();

  private final CommandXboxController driverController = new CommandXboxController(0);

  public RobotContainer() {
    drivetrainSubsystem.setDefaultCommand(drivetrainSubsystem.arcadeDriveCommand(
      () -> driverController.getLeftY(),
      () -> driverController.getLeftX()
    ));

    configureBindings();
  }

  private void configureBindings() {
    driverController.a().onTrue(Commands.print("The A button was pressed!"));
  }

  public Command getAutonomousCommand() {
    return new DriveDistancePID(drivetrainSubsystem, 5);
  }
}
