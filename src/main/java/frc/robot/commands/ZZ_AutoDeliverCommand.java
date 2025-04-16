// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CoralOutakeSubSystem;
import frc.robot.subsystems.ElevatorSubSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZZ_AutoDeliverCommand extends SequentialCommandGroup {
  /** Creates a new AutoDeliverCommand. */
  public ZZ_AutoDeliverCommand(Command goToLevelCommand, ElevatorSubSystem elevator, CoralOutakeSubSystem coralOutake, double shootAtHeight) {
    addCommands(
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
            new WaitUntilAtHeightCommand(elevator, shootAtHeight),
            new ShootCoralCommand(coralOutake)), 
        goToLevelCommand),
      new ElevatorDownCommand(elevator)
    );
    addRequirements(elevator,coralOutake);
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    //super(new SequentialCommandGroup(new AtHeightCommand(elevator, shootAtHeight), new ShootCoralCommand(coralOutake)));
    // addCommands(new FooCommand(), new BarCommand());
    //addCommands(goToLevelCommand);
  }
}
