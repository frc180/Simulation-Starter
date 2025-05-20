// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  private double leftSpeedTarget = 0;
  private double rightSpeedTarget = 0;
  public PIDController drivePid, turnPid;

  // SIMULATION VARIABLES
  private DifferentialDrivetrainSim drivetrainSim;
  private Field2d field2d;

  public Drivetrain() {
    initSimulation();
    drivePid = new PIDController(1, 0, 0.14);
    turnPid = new PIDController(0.2, 0, 0.025);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Convert our [-1, 1] speed targets to voltages for the drivetrain
    drivetrainSim.setInputs(leftSpeedTarget * 12, rightSpeedTarget * 12);
  }

  /**
   * Sets the speed of the left and right sides of the Drivetrain.
   * @param leftSpeed Speed of the left side, ranging -1 to 1.
   * @param rightSpeed Speed of the right side, ranging -1 to 1.
   */
  public void setSpeed(double leftSpeed, double rightSpeed) {
    leftSpeedTarget = minMax(leftSpeed, -1, 1);
    rightSpeedTarget = minMax(rightSpeed, -1, 1);
  }

  public void arcadeDrive(double forward, double turn) {
    setSpeed(forward + turn, forward - turn);
  }

  public Command arcadeDriveCommand(DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed) {
    return Commands.run(() -> {
      arcadeDrive(forwardSpeed.getAsDouble(), turnSpeed.getAsDouble());
    }, this);
  }
  
  /**
   * Ensures a value is within a minimum and maximum value.
   * @param value The value to be constrained.
   * @param min The minimum the value must be.
   * @param max The maximum the value must be.
   * @return The value itself if it is within the min and max - otherwise, the min or max itself.
   */
  public double minMax(double value, double min, double max) {
    if (value < min) {
      return min;
    }
    if (value > max) {
      return max;
    }
    return value;
  }

  public Rotation2d getGyro() {
    return drivetrainSim.getHeading();
  }

  public Pose2d getPose() {
    return drivetrainSim.getPose();
  }

  public void setPose(Pose2d pose) {
    drivetrainSim.setPose(pose);
  }

  public double getLeftEncoderDistance() {
    return drivetrainSim.getLeftPositionMeters();
  }

  public double getLeftEncoderVelocity() {
    return drivetrainSim.getLeftVelocityMetersPerSecond();
  }

  public double getRightEncoderDistance() {
    return drivetrainSim.getRightPositionMeters();
  }

  public double getRightEncoderVelocity() {
    return drivetrainSim.getRightVelocityMetersPerSecond();
  }

  /* ======================= SIMULATION METHODS =======================
   * Do not add, remove or change code here to accomplish tasks.
   */ 

  public void initSimulation() {
    drivetrainSim = DifferentialDrivetrainSim.createKitbotSim(
      KitbotMotor.kDoubleNEOPerSide,
      KitbotGearing.k7p31,
      KitbotWheelSize.kSixInch, 
      3,
      null
    );
    field2d = new Field2d();
    SmartDashboard.putData("Field", field2d);
  }

  @Override
  public void simulationPeriodic() {
    drivetrainSim.update(0.02);
    field2d.setRobotPose(drivetrainSim.getPose());
  }

  public void setTarget(Pose2d target) {
    field2d.getObject("Target").setPose(target);
  }
}
