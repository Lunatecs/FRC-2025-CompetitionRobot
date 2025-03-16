// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathFindToPose extends Command {
  /** Creates a new PathFindToPose. */
  CommandSwerveDrivetrain swerve;
  Pose2d Tag18 = new Pose2d(3.26, 4.20, new Rotation2d(0));
  Pose2d currentPose;
  public PathFindToPose(CommandSwerveDrivetrain swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPose = swerve.getState().Pose;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    try {
      PathPlannerPath path = new PathPlannerPath(PathPlannerPath.waypointsFromPoses(currentPose, Tag18), new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), null, new GoalEndState(0, new Rotation2d(0)));
      path.preventFlipping = true;
      AutoBuilder.followPath(path);
    } catch (Exception e) {
      e.printStackTrace();
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
