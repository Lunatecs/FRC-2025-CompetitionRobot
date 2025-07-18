// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CoralOutakeSubSystem;
import frc.robot.subsystems.ElevatorSubSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZZ_Deliver extends SequentialCommandGroup {
  /** Creates a new Deliver. */
  public ZZ_Deliver(Command goToLevelCommand, ElevatorSubSystem elevator, CoralOutakeSubSystem coralOutake, double shootAtHeight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
            new WaitUntilAtHeightCommand(elevator, shootAtHeight),
            new ShootCoralCommand(coralOutake)), 
        goToLevelCommand)
    );
    addRequirements(elevator,coralOutake);
  }
}
