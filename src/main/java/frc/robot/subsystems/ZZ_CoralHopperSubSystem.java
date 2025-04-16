// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ZZ_CoralHopperSubSystem extends SubsystemBase {
  public TalonFX hopperMotor;
  /** Creates a new CoralHopperSubSystem. */
  public ZZ_CoralHopperSubSystem() {
    hopperMotor=new TalonFX(2607);
    hopperMotor.setNeutralMode(NeutralModeValue.Coast);
    hopperMotor.setInverted(true);
  }
  public void setSpeed(double speed){
    hopperMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
