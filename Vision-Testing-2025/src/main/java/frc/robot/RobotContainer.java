// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Commands.AbsoluteDriveAdv;
import frc.robot.Subsystems.Drivetrain.Drivetrain;

public class RobotContainer {
  private final CommandXboxController driver;
  private final Drivetrain drivetrain;

  
  AbsoluteDriveAdv closedAbsoluteDriveAdv;

  public RobotContainer() {
    driver = new CommandXboxController(0);
    drivetrain = new Drivetrain(new File(Filesystem.getDeployDirectory(),"swerve"));
    new AbsoluteDriveAdv(drivetrain,
                                                                 () -> -MathUtil.applyDeadband(driver.getLeftY(),
                                                                                               OperatorConstants.LEFT_Y_DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driver.getLeftX(),
                                                                                               OperatorConstants.DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driver.getRightX(),
                                                                                               OperatorConstants.RIGHT_X_DEADBAND),
                                                                 driver.getHID()::getYButtonPressed,
                                                                 driver.getHID()::getAButtonPressed,
                                                                 driver.getHID()::getXButtonPressed,
                                                                 driver.getHID()::getBButtonPressed);
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
