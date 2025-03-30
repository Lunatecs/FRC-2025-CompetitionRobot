// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralAlignmentSubSystem;
import frc.robot.subsystems.LEDSubSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SensorAlignToPole extends Command {
  /** Creates a new LEDAlignL4. */
  LEDSubSystem led;
  CoralAlignmentSubSystem align;
  boolean isFinished;
  int level;
  public SensorAlignToPole(int level) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.level = level;
    addRequirements(align);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (level == 2 && align.isAlignedL2()) {
      led.setColor(led.SOLID_GREEN);
      isFinished = true;
    }

    if (level == 3 && align.isAlignedL3()) {
      led.setColor(led.SOLID_GREEN);
      isFinished = true;
    }

    if (level == 4 && align.isAlignedL4()) {
      led.setColor(led.SOLID_GREEN);
      isFinished = true;
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
