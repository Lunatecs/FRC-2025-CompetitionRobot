// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralGroundIntakePivotSubSystem extends SubsystemBase {
  private final double gearRatio = 25.0;//25/1
  private final double rotationsToDegrees = 360.0/gearRatio;
  /** Creates a new CoralGroundIntakePivotSubSystem. */
  private TalonFX pivotMotor;

  public CoralGroundIntakePivotSubSystem() {
    pivotMotor = new TalonFX(Constants.CoralGroundIntakePivotSubSystemConstants.CAN_ID_INTAKE_PIVOT);
  }


  public void setSpeed(double speed) {
    pivotMotor.set(speed);
  }

  public double getEncoder(){
    return pivotMotor.getPosition().getValueAsDouble();
  }

  public double getDegreesOfPivot(){
    return getEncoder()* rotationsToDegrees;
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
