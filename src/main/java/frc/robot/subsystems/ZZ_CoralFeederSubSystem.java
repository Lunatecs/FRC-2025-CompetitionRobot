// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ZZ_CoralFeederSubSystem extends SubsystemBase {
  /** Creates a new CoralFeederSubSystem. */

  public SparkMax motor = new SparkMax(Constants.CoralFeederSubSystemConstants.CAN_ID_FEEDER,SparkLowLevel.MotorType.kBrushless);

  public ZZ_CoralFeederSubSystem() {
    motor.configure(new SparkMaxConfig().inverted(true), SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
