// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CoralOutakeSubSystem;
import frc.robot.subsystems.ElevatorSubSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FullAutoDeliverCoralTeleop extends SequentialCommandGroup {
  /** Creates a new FullAutoDeliverCoralTeleop. */
  public FullAutoDeliverCoralTeleop(int level, Command goToLevelCommand, ElevatorSubSystem elevator, CoralOutakeSubSystem coralOutake, double shootAtHeight) {
    addCommands(new AutoDeliverCoralTeleop(level, goToLevelCommand, elevator, coralOutake, shootAtHeight),
    new ElevatorDownCommand(elevator));
  }
}
