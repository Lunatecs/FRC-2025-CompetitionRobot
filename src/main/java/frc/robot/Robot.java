// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Rotation2d autonEndingAngle = new Rotation2d();

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override public void driverStationConnected() {

      double targetRotationDegrees = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) ? 0 : 180;
      m_robotContainer.drivetrain.resetRotation(Rotation2d.fromDegrees(targetRotationDegrees));
  }


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 

    try {
      var driveState = m_robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      LimelightHelpers.SetRobotOrientation("limelight-left", headingDeg, 0, 0, 0, 0, 0);
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
      if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
      }

      SmartDashboard.putString("Limelight swerve pose", llMeasurement.pose.toString());
    } catch(Exception e) {
            DriverStation.reportError("Error getting POSE from LIMELIGHT (check ethernet): " + e.getMessage(), e.getStackTrace());
    }
  }
/*
  @Override
  public void robotInit() {
    //super.robotInit();
    //FollowPathCommand.warmupCommand().schedule();
  } */

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    m_robotContainer.dropWhaleTale();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    autonEndingAngle = m_robotContainer.drivetrain.getState().Pose.getRotation();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.drivetrain.resetRotation(autonEndingAngle.plus(Rotation2d.fromDegrees(180)));
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

  @Override
  public void simulationPeriodic() {}
}
