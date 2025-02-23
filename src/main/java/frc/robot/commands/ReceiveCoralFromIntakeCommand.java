// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CarriageSubSystem;
import frc.robot.subsystems.CoralFeederSubSystem;
import frc.robot.subsystems.CoralGroundIntakeSubSystem;
import frc.robot.subsystems.CoralOutakeSubSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReceiveCoralFromIntakeCommand extends Command {
  private CoralOutakeSubSystem outake;
  private CarriageSubSystem carriage;
  private CoralFeederSubSystem feeder;
  private CoralGroundIntakeSubSystem intake;
  private boolean isFinished;

  /** Creates a new ReceiveCoralCommand. */
  public ReceiveCoralFromIntakeCommand(CoralOutakeSubSystem outake, CarriageSubSystem carriage, CoralFeederSubSystem feeder, CoralGroundIntakeSubSystem intake) {
    this.outake = outake;
    this.carriage = carriage;
    this.feeder = feeder;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(outake, carriage, feeder, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(outake.hasCoral()){
      isFinished = true;
      outake.setSpeed(0);
      carriage.setSpeed(0);
      feeder.setSpeed(0);
      intake.setSpeed(0);
    }
    else{
      outake.setSpeed(1);
      carriage.setSpeed(1);
      feeder.setSpeed(1);
      intake.setSpeed(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      outake.setSpeed(0);
      carriage.setSpeed(0);
      feeder.setSpeed(0);
      intake.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
