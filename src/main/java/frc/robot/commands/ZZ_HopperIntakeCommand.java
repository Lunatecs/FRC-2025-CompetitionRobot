// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ZZ_CoralHopperSubSystem;
import frc.robot.subsystems.CarriageSubSystem;
import frc.robot.subsystems.CoralOutakeSubSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZZ_HopperIntakeCommand extends Command {
  /** Creates a new HopperIntakeCommand. */
  private ZZ_CoralHopperSubSystem hopper;
  private CarriageSubSystem carriage;
  private CoralOutakeSubSystem outake;
  private boolean isFinished;

  public ZZ_HopperIntakeCommand(ZZ_CoralHopperSubSystem hopper, CarriageSubSystem carriage, CoralOutakeSubSystem outake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hopper = hopper;
    this.carriage = carriage;
    this.outake = outake;

    addRequirements(hopper, carriage, outake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(outake.hasCoral()) {
      isFinished = true;
      hopper.setSpeed(0);
      carriage.setSpeed(0);
      outake.setSpeed(0);
    }
    else{
      hopper.setSpeed(1);
      carriage.setSpeed(1);
      outake.setSpeed(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.setSpeed(0);
    carriage.setSpeed(0);
    outake.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
