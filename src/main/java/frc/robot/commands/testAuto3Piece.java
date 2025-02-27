// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralOutakeSubSystem;
import frc.robot.subsystems.ElevatorSubSystem;
import frc.robot.subsystems.ScoringLimeLightSubSystemLeft;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class testAuto3Piece extends SequentialCommandGroup {
  /** Creates a new testAuto3Piece. */
  public testAuto3Piece(PathPlannerPath path, ScoringLimeLightSubSystemLeft limelightLeft, CommandSwerveDrivetrain drivetrain, ElevatorSubSystem elevator, CoralOutakeSubSystem coralOutake, SwerveRequest.RobotCentric robotCentric, double MaxSpeed, double MaxAngularRate) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      AutoBuilder.followPath(path),
      new AutoTargetScoreRightPoleL4Sequence(limelightLeft, drivetrain, elevator, coralOutake, robotCentric, MaxSpeed, MaxAngularRate)
    );
  }
}
