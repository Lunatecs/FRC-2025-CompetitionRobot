// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AprilTagPositions;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefTrackingCommand extends Command {
  /** Creates a new ReefTrackingCommand. */
  private CommandSwerveDrivetrain drive;
  private Command fullPath;

  public ReefTrackingCommand(CommandSwerveDrivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
      Pose2d closestAprilTagPose = getClosestReefAprilTagPose();
      PathPlannerPath path = new PathPlannerPath(PathPlannerPath.waypointsFromPoses(translateCoord(closestAprilTagPose, closestAprilTagPose.getRotation().getDegrees(), -0.1)), null, null, new GoalEndState(0.0, closestAprilTagPose.getRotation()));
      path.preventFlipping = true;
      fullPath = AutoBuilder.followPath(path);
      fullPath.schedule();
    } catch (Exception e) {
      DriverStation.reportError("Reef Tracking Error: " + e.getMessage(), e.getStackTrace());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (fullPath != null) {
      fullPath.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private Pose2d getClosestReefAprilTagPose() {
    HashMap<Integer, Pose2d> aprilTagsToAlignTo = AprilTagPositions.WELDED_BLUE_CORAL_APRIL_TAG_POSITIONS;
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        aprilTagsToAlignTo = AprilTagPositions.WELDED_RED_CORAL_APRIL_TAG_POSITIONS;
      }
    }

    Pose2d currentPose = drive.getState().Pose;
    Pose2d closestPose = new Pose2d();
    double closestDistance = Double.MAX_VALUE;
    Integer aprilTagNum = -1;

    for (Map.Entry<Integer, Pose2d> entry : aprilTagsToAlignTo.entrySet()) {
      Pose2d pose = entry.getValue();
      double distance = findDistanceBetween(currentPose, pose);
      if (distance < closestDistance) {
        closestDistance = distance;
        closestPose = pose;
        aprilTagNum = entry.getKey();
      }
    }

    Pose2d inFrontOfAprilTag = translateCoord(closestPose, closestPose.getRotation().getDegrees(),
        -Units.inchesToMeters(18));

    return translateCoord(inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 90, 0.2); //0.2 is distance in meters from left limelight to endeffector
  } 

  private Pose2d translateCoord(Pose2d originalPose, double degreesRotate, double distance) {
    double newXCoord = originalPose.getX() + (Math.cos(Math.toRadians(degreesRotate)) * distance);
    double newYCoord = originalPose.getY() + (Math.sin(Math.toRadians(degreesRotate)) * distance);

    return new Pose2d(newXCoord, newYCoord, originalPose.getRotation());
  }

  private double findDistanceBetween(Pose2d pose1, Pose2d pose2) {
    return Math.sqrt(Math.pow((pose2.getX() - pose1.getX()), 2) + Math.pow((pose2.getY() - pose1.getY()), 2));
  }
}
