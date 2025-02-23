// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CarriageSubSystem;
import frc.robot.subsystems.CoralFeederSubSystem;
import frc.robot.subsystems.CoralGroundIntakePivotSubSystem;
import frc.robot.subsystems.CoralGroundIntakeSubSystem;
import frc.robot.subsystems.CoralOutakeSubSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeCoralCommand extends SequentialCommandGroup {
  /** Creates a new IntakeCoralCommand. */
  public IntakeCoralCommand(CoralGroundIntakePivotSubSystem pivot, CoralGroundIntakeSubSystem intake, CoralFeederSubSystem feeder, CarriageSubSystem carriage, CoralOutakeSubSystem outake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelDeadlineGroup(new ReceiveCoralFromIntakeCommand(outake, carriage, feeder, intake), new DropIntakeCommand(pivot)),
    new RaiseIntakeCommand(pivot)
    );
    addRequirements(pivot, intake, carriage, feeder, outake);
  }
}
