// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.GoTo;
import frc.souffle.Souffle;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    Photon.getInstance().update();
    CommandScheduler.getInstance().run();
    Souffle.update();
  }

  @Override
  public void disabledInit() {
    LimelightHelpers.getLimelightNTTableEntry(Constants.ReefLimelightName, "throttle_set").setDouble(150);
    LimelightHelpers.getLimelightNTTableEntry(Constants.UpperLimelightName, "throttle_set").setDouble(150);
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    LimelightHelpers.getLimelightNTTableEntry(Constants.ReefLimelightName, "throttle_set").setDouble(0);
    LimelightHelpers.getLimelightNTTableEntry(Constants.UpperLimelightName, "throttle_set").setDouble(0);

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    m_robotContainer.setStartingPose();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    LimelightHelpers.getLimelightNTTableEntry(Constants.ReefLimelightName, "throttle_set").setDouble(0);
    LimelightHelpers.getLimelightNTTableEntry(Constants.UpperLimelightName, "throttle_set").setDouble(0);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    System.err.println("Got allaince " + GoTo.getAlliance().toString());
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
