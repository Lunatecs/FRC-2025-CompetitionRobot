// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;


public class AlgaeLiberatorSubSystem extends SubsystemBase {
  /** Creates a new AlgaeLiberatorSubSystem. */

  public TalonFX intakeMotor;

  public AlgaeLiberatorSubSystem() {
    intakeMotor = new TalonFX(Constants.AlgaeLiberatorSubSystemConstants.CAN_ID_ALGAE_LIBERATOR);
  }

  public void setSpeed(double speed){
    intakeMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
