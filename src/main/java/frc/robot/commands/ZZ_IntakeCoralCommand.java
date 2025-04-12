// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CarriageSubSystem;
import frc.robot.subsystems.ZZ_CoralFeederSubSystem;
import frc.robot.subsystems.ZZ_CoralGroundIntakePivotSubSystem;
import frc.robot.subsystems.ZZ_CoralGroundIntakeSubSystem;
import frc.robot.subsystems.CoralOutakeSubSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZZ_IntakeCoralCommand extends SequentialCommandGroup {
  /** Creates a new IntakeCoralCommand. */
  public ZZ_IntakeCoralCommand(ZZ_CoralGroundIntakePivotSubSystem pivot, ZZ_CoralGroundIntakeSubSystem intake, ZZ_CoralFeederSubSystem feeder, CarriageSubSystem carriage, CoralOutakeSubSystem outake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelDeadlineGroup(new ZZ_ReceiveCoralFromIntakeCommand(outake, carriage, feeder, intake), new ZZ_DropIntakeCommand(pivot)),
    new ZZ_RaiseIntakeCommand(pivot)
    );
    addRequirements(pivot, intake, carriage, feeder, outake);
  }
}
