// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Robot extends TimedRobot {
  private int AUTONOMOUS_LEVEL = 1;

  private Command m_autonomousCommand;
  private Timer autoTimer = new Timer();

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // Run the supplied autonomous mode and evaluate how close it got to the target
    Translation2d autoTranslate = null;
    switch (AUTONOMOUS_LEVEL) {
      case 1:
        autoTranslate = new Translation2d(5, 0);
        break;
      case 2:
        autoTranslate = new Translation2d(7, 3);
        break;
    }

    Pose2d start = new Pose2d(2, 2, new Rotation2d());
    Pose2d target = start.plus(new Transform2d(autoTranslate, new Rotation2d()));
    m_robotContainer.drivetrainSubsystem.setPose(start);
    m_robotContainer.drivetrainSubsystem.setTarget(target);

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      Commands.sequence(
        Commands.runOnce(autoTimer::restart),
        m_autonomousCommand,
        new WaitCommand(1),
        Commands.runOnce(() -> {
          Pose2d diff = m_robotContainer.drivetrainSubsystem.getPose().relativeTo(target);
          System.out.println("========= Autonomous Results =========");
          System.out.println(String.format("%.3g seconds taken", autoTimer.get() - 1));
          System.out.println(String.format("X error: %.4g m%nY error: %.4g m", diff.getX(), diff.getY()));
        })
      ).schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
