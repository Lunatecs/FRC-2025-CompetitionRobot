// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ScoringLimeLightSubSystemRight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FullAlignRightLimeLight extends SequentialCommandGroup {
  /** Creates a new FullAlignRight. */
  public FullAlignRightLimeLight(ScoringLimeLightSubSystemRight limelightRight, CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric robotCentric, double MaxSpeed, double MaxAngularRate) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AlignRobotToTagRightLimeLight(limelightRight, drivetrain, robotCentric, MaxSpeed, MaxAngularRate),
      new ParallelDeadlineGroup(new WaitCommand(1.5), new InstantCommand(() -> drivetrain.setControl(robotCentric.withVelocityX(1.0).withVelocityY(0.0))))
    );
  }
}
