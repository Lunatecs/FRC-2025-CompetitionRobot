// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToReefPoseCommand extends Command {
  /** Creates a new AlignToReefPoseCommand. */
  private CommandSwerveDrivetrain swerve;
  Pose2d currentPose;
  Pose2d goalPose;
  List<Waypoint> waypoints;
  PathConstraints constraints;
  PathPlannerPath path;
  Command run;
  public AlignToReefPoseCommand(CommandSwerveDrivetrain swerve) {
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPose = swerve.getState().Pose;
    goalPose = new Pose2d(3.161, 3.850, Rotation2d.fromDegrees(180));
    SmartDashboard.putString("goal pose", goalPose.toString());
    waypoints = PathPlannerPath.waypointsFromPoses(currentPose, goalPose);
    constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
    path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0, goalPose.getRotation()));
    path.preventFlipping = true;
    run = AutoBuilder.followPath(path); 
    run.initialize();
    
  }
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    run.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    run.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return run.isFinished();
  }
}
