// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZZ_PathFindToPose extends Command {
  /** Creates a new PathFindToPose. */
  CommandSwerveDrivetrain swerve;
  //Pose2d Tag18 = new Pose2d(3.26, 4.20, new Rotation2d(0));
  Pose2d endPose;
  Pose2d currentPose;
  PathPlannerPath path;
  Command run;


  

  public ZZ_PathFindToPose(CommandSwerveDrivetrain swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        new Pose2d(1.0, 0, Rotation2d.fromDegrees(0)));
      
    PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage
        
        // Create the path using the waypoints created above
    PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );
        
        // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = true;
    /*currentPose = swerve.getState().Pose;
    endPose = new Pose2d(swerve.getState().Pose.getX() + 1, swerve.getState().Pose.getY(), new Rotation2d(0));
    path = new PathPlannerPath(PathPlannerPath.waypointsFromPoses(currentPose, endPose), new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), null, new GoalEndState(0, new Rotation2d(0)));
    path.preventFlipping = true;*/
    run = AutoBuilder.followPath(path);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      run.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return run.isFinished();
  }
}
